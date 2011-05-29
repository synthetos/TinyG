# -*- coding: utf-8 -*-
"""
$Id: threading.py 783 2010-10-16 02:08:42Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>

This file is part of PyCAM.

PyCAM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PyCAM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with PyCAM.  If not, see <http://www.gnu.org/licenses/>.
"""

import pycam.Utils.log
# multiprocessing is imported later
#import multiprocessing
#from multiprocessing.managers import SyncManager
import Queue
import platform
import random
import uuid
import time
import os

DEFAULT_PORT = 1250


log = pycam.Utils.log.get_logger()

#TODO: create one or two classes for these functions (to get rid of the globals)

# possible values:
#   None: not initialized
#   False: no threading
#   multiprocessing: the multiprocessing module is impored and enabled
__multiprocessing = None

# needs to be initialized, if multiprocessing is enabled
__num_of_processes = None

__manager = None
__closing = None
__task_source_uuid = None
__finished_jobs = []


def run_in_parallel(*args, **kwargs):
    global __manager
    if __manager is None:
        return run_in_parallel_local(*args, **kwargs)
    else:
        return run_in_parallel_remote(*args, **kwargs)

def is_pool_available():
    return not __manager is None

def get_pool_statistics():
    global __manager
    if __manager is None:
        return []
    else:
        return __manager.statistics().get_worker_statistics()

def init_threading(number_of_processes=None, enable_server=False, remote=None, run_server=False,
        server_credentials=""):
    global __multiprocessing, __num_of_processes, __manager, __closing, __task_source_uuid
    # only local -> no server settings allowed
    if (not enable_server) and (not run_server):
        remote = None
        run_server = None
        server_credentials = ""
    try:
        import multiprocessing
        mp_is_available = True
    except ImportError:
        mp_is_available = False
    if not mp_is_available:
        __multiprocessing = False
        # Maybe a multiprocessing feature was explicitely requested?
        # Issue some warnings if necessary.
        multiprocessing_missing_text = "Failed to enable server mode due to " \
                + "a lack of 'multiprocessing' capabilities. Please use " \
                + "Python2.6 or install the 'python-multiprocessing' package."
        if enable_server:
            log.warn("Failed to enable server mode due to a lack of " \
                    + "'multiprocessing' capabilities. " \
                    + multiprocessing_missing_text)
        elif run_server:
            log.warn("Failed to run in server-only mode due to a lack of " \
                    + "'multiprocessing' capabilities. " \
                    + multiprocessing_missing_text)
        else:
            # no further warnings required
            pass
    else:
        if number_of_processes is None:
            # use defaults
            # don't enable threading for a single cpu
            if (multiprocessing.cpu_count() > 1) or remote or run_server or enable_server:
                __multiprocessing = multiprocessing
                __num_of_processes = multiprocessing.cpu_count()
            else:
                __multiprocessing = False
        elif (number_of_processes < 1) and (remote is None) and (enable_server is None):
            # zero processes are allowed if we use a remote server or offer a server
            __multiprocessing = False
        else:
            __multiprocessing = multiprocessing
            __num_of_processes = number_of_processes
    # initialize the manager
    if not __multiprocessing:
        __manager == None
        log.info("Disabled parallel processing")
    elif not enable_server and not run_server:
        __manager == None
        log.info("Enabled %d parallel local processes" % __num_of_processes)
    else:
        # with multiprocessing
        log.info("Enabled %d parallel local processes" % __num_of_processes)
        log.info("Allow remote processing")
        # initialize the uuid list for all workers
        worker_uuid_list = [str(uuid.uuid1()) for index in range(__num_of_processes)]
        __task_source_uuid = str(uuid.uuid1())
        if remote is None:
            address = ('', DEFAULT_PORT)
        else:
            if ":" in remote:
                host, port = remote.split(":", 1)
                try:
                    port = int(port)
                except ValueError:
                    log.warning(("Invalid port specified: '%s' - using default " \
                            + "port (%d) instead") % (port, DEFAULT_PORT))
                    port = DEFAULT_PORT
            else:
                host = remote
                port = DEFAULT_PORT
            address = (host, port)
        from multiprocessing.managers import SyncManager
        class TaskManager(SyncManager):
            @classmethod
            def _run_server(cls, *args):
                # make sure that the server ignores SIGINT (KeyboardInterrupt)
                import signal
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                SyncManager._run_server(*args)
        if remote is None:
            tasks_queue = multiprocessing.Queue()
            results_queue = multiprocessing.Queue()
            statistics = ProcessStatistics()
            cache = ProcessDataCache()
            TaskManager.register("tasks", callable=lambda: tasks_queue)
            TaskManager.register("results", callable=lambda: results_queue)
            TaskManager.register("statistics", callable=lambda: statistics)
            TaskManager.register("cache", callable=lambda: cache)
        else:
            TaskManager.register("tasks")
            TaskManager.register("results")
            TaskManager.register("statistics")
            TaskManager.register("cache")
        __manager = TaskManager(address=address, authkey=server_credentials)
        # run the local server, connect to a remote one or begin serving
        if remote is None:
            __manager.start()
            log.info("Started a local server.")
        else:
            __manager.connect()
            log.info("Connected to a remote task server.")
        # create the spawning process
        __closing = __manager.Value("b", False)
        if __num_of_processes > 0:
            # only start the spawner, if we want to use local workers
            spawner = __multiprocessing.Process(name="spawn", target=_spawn_daemon,
                    args=(__manager, __num_of_processes, worker_uuid_list))
            spawner.start()
        else:
            spawner = None
        # wait forever - in case of a server
        if run_server:
            log.info("Running a local server and waiting for remote connections.")
            # the server can be stopped via CTRL-C - it is caught later
            if not spawner is None:
                spawner.join()

def cleanup():
    global __manager, __closing
    if __multiprocessing and __closing:
        log.debug("Shutting down process handler")
        try:
            __closing.set(True)
        except EOFError:
            log.debug("Connection to manager lost during cleanup")
        # Only managers that were started via ".start()" implement a "shutdown".
        # Managers started via ".connect" may skip this.
        if hasattr(__manager, "shutdown"):
            # wait for the spawner and the worker threads to go down
            time.sleep(2.5)
            #__manager.shutdown()
            time.sleep(0.1)
            # check if it is still alive and kill it if necessary
            if __manager._process.is_alive():
                __manager._process.terminate()

def _spawn_daemon(manager, number_of_processes, worker_uuid_list):
    """ wait for items in the 'tasks' queue to appear and then spawn workers
    """
    global __multiprocessing, __closing
    tasks = manager.tasks()
    results = manager.results()
    stats = manager.statistics()
    cache = manager.cache()
    log.debug("Spawner daemon started with %d processes" % number_of_processes)
    log.debug("Registering %d worker threads: %s" \
            % (len(worker_uuid_list), worker_uuid_list))
    last_cache_update = time.time()
    # use only the hostname (for brevity) - no domain part
    hostname = platform.node().split(".", 1)[0]
    try:
        while not __closing.get():
            # check the expire timeout of the cache from time to time
            if last_cache_update + 30 < time.time():
                cache.expire_cache_items()
                last_cache_update = time.time()
            if not tasks.empty():
                workers = []
                for task_id in worker_uuid_list:
                    task_name = "%s-%s" % (hostname, task_id)
                    worker = __multiprocessing.Process(
                            name=task_name, target=_handle_tasks,
                            args=(tasks, results, stats, cache,
                            __closing))
                    worker.start()
                    workers.append(worker)
                # wait until all workers are finished
                for worker in workers:
                    worker.join()
            else:
                time.sleep(1.0)
    except KeyboardInterrupt:
        log.info("Spawner daemon killed by keyboard interrupt")
        # set the "closing" flag and just exit
        __closing.set(True)
    except EOFError:
        # the connection was closed
        log.info("Spawner daemon lost connection to server")

def _handle_tasks(tasks, results, stats, cache, closing):
    global __multiprocessing
    name = __multiprocessing.current_process().name
    local_cache = ProcessDataCache()
    timeout_limit = 60
    timeout_counter = 0
    last_worker_notification = 0
    log.debug("Worker thread started: %s" % name)
    try:
        while (timeout_counter < timeout_limit) and not closing.get():
            if last_worker_notification + 30 < time.time():
                stats.worker_notification(name)
                last_worker_notification = time.time()
            try:
                start_time = time.time()
                job_id, task_id, func, args = tasks.get(timeout=1.0)
                # reset the timeout counter, if we found another item in the queue
                timeout_counter = 0
                real_args = []
                for arg in args:
                    if isinstance(arg, ProcessDataCacheItemID):
                        try:
                            value = local_cache.get(arg)
                        except KeyError:
                            # TODO: we will break hard, if the item is expired
                            value = cache.get(arg)
                            local_cache.add(arg, value)
                        real_args.append(value)
                    elif isinstance(arg, list) and [True for item in arg \
                            if isinstance(item, ProcessDataCacheItemID)]:
                        # check if any item in the list is cacheable
                        args_list = []
                        for item in arg:
                            if isinstance(item, ProcessDataCacheItemID):
                                try:
                                    value = local_cache.get(item)
                                except KeyError:
                                    value = cache.get(item)
                                    local_cache.add(item, value)
                                args_list.append(value)
                            else:
                                args_list.append(item)
                        real_args.append(args_list)
                    else:
                        real_args.append(arg)
                stats.add_transfer_time(name, time.time() - start_time)
                start_time = time.time()
                results.put((job_id, task_id, func(real_args)))
                stats.add_process_time(name, time.time() - start_time)
            except Queue.Empty:
                time.sleep(1.0)
                timeout_counter += 1
    except KeyboardInterrupt:
        pass
    log.debug("Worker thread finished after %d seconds of inactivity: %s" \
            % (timeout_counter, name))

def run_in_parallel_remote(func, args_list, unordered=False,
        disable_multiprocessing=False):
    global __multiprocessing, __num_of_processes, __manager, __task_source_uuid, __finished_jobs
    if __multiprocessing is None:
        # threading was not configured before
        init_threading()
    if __multiprocessing and not disable_multiprocessing:
        job_id = str(uuid.uuid1())
        log.debug("Starting parallel tasks: %s" % job_id)
        tasks_queue = __manager.tasks()
        results_queue = __manager.results()
        remote_cache = __manager.cache()
        stats = __manager.statistics()
        for index, args in enumerate(args_list):
            start_time = time.time()
            result_args = []
            for arg in args:
                # add the argument to the cache if possible
                if hasattr(arg, "uuid"):
                    data_uuid = ProcessDataCacheItemID(arg.uuid)
                    if not remote_cache.contains(data_uuid):
                        log.debug("Adding cache item for job %s: %s - %s" % (job_id, arg.uuid, arg.__class__))
                        remote_cache.add(data_uuid, arg)
                    result_args.append(data_uuid)
                elif isinstance(arg, (list, set, tuple)) \
                        and ([True for item in arg if hasattr(item, "uuid")]):
                    # a list with at least one cacheable item
                    new_arg_list = []
                    for item in arg:
                        if hasattr(item, "uuid"):
                            data_uuid = ProcessDataCacheItemID(item.uuid)
                            if not remote_cache.contains(data_uuid):
                                log.debug("Adding cache item from list for " \
                                        + "job %s: %s - %s" \
                                        % (job_id, item.uuid, item.__class__))
                                remote_cache.add(data_uuid, item)
                            new_arg_list.append(data_uuid)
                        else:
                            new_arg_list.append(item)
                    result_args.append(new_arg_list)
                else:
                    result_args.append(arg)
            tasks_queue.put((job_id, index, func, result_args))
            stats.add_queueing_time(__task_source_uuid, time.time() - start_time)
        log.debug("Added %d tasks for job %s" % (len(args_list), job_id))
        result_buffer = {}
        index = 0
        while index < len(args_list):
            try:
                result_job_id = None
                while result_job_id != job_id:
                    result_job_id, task_id, result = results_queue.get()
                    if result_job_id == job_id:
                        if unordered:
                            # just return the values in any order
                            yield result
                            index += 1
                        else:
                            # return the results in order (based on task_id)
                            if task_id == index:
                                yield result
                                index += 1
                                while index in result_buffer.keys():
                                    yield result_buffer[index]
                                    del result_buffer[index]
                                    index += 1
                            else:
                                result_buffer[task_id] = result
                    elif result_job_id in __finished_jobs:
                        # throw away this result of an old job
                        log.debug("Throwing away a result of an old task: %s" % result_job_id)
                        pass
                    else:
                        log.debug("Skipping result of non-local task: %s" % result_job_id)
                        # put the result back to the queue for the next manager
                        results_queue.put((result_job_id, task_id, result))
                        # wait for up to 0.2 seconds before trying again
                        time.sleep(random.random() / 5)
            except GeneratorExit:
                log.debug("Parallel processing canceled: %s" % job_id)
                # catch this specific (silent) exception and flush the task queue
                queue_len = tasks_queue.qsize()
                # remove all remaining tasks with the current job id
                removed_job_counter = 0
                for index in range(queue_len):
                    this_job_id, task_id, func, args = tasks_queue.get(timeout=0.1)
                    if this_job_id != job_id:
                        tasks_queue.put((this_job_id, task_id, func, args))
                    else:
                        removed_job_counter += 1
                if removed_job_counter > 0:
                    log.debug("Removed %d remaining tasks for %s" % (removed_job_counter, job_id))
                __finished_jobs.append(job_id)
                # don't keep more than 10 old job ids
                while len(__finished_jobs) > 10:
                    __finished_jobs.pop(0)
                # re-raise the GeneratorExit exception to finish destruction
                raise
        log.debug("Parallel processing finished: %s" % job_id)
    else:
        for args in args_list:
            yield func(args)

def run_in_parallel_local(func, args, unordered=False, disable_multiprocessing=False):
    global __multiprocessing, __num_of_processes
    if __multiprocessing is None:
        # threading was not configured before
        init_threading()
    if __multiprocessing and not disable_multiprocessing:
        # use the number of CPUs as the default number of worker threads
        pool = __multiprocessing.Pool(__num_of_processes)
        if unordered:
            imap_func = pool.imap_unordered
        else:
            imap_func = pool.imap
        # Beware: we may not return "pool.imap" or "pool.imap_unordered"
        # directly. It would somehow loose the focus and just hang infinitely.
        # Thus we wrap our own generator around it.
        for result in imap_func(func, args):
            yield result
    else:
        for arg in args:
            yield func(arg)


class OneProcess(object):
    def __init__(self, name, is_queue=False):
        self.is_queue = is_queue
        self.name = name
        self.transfer_time = 0
        self.transfer_count = 0
        self.process_time = 0
        self.process_count = 0

    def __str__(self):
        try:
            if self.is_queue:
                return "Queue %s: %s (%s/%s)" \
                    % (self.name, self.transfer_time/self.transfer_count,
                            self.transfer_time, self.transfer_count)
            else:
                return "Process %s: %s (%s/%s) - %s (%s/%s)" \
                        % (self.name, self.transfer_time/self.transfer_count,
                                self.transfer_time, self.transfer_count,
                                self.process_time/self.process_count,
                                self.process_time, self.process_count)
        except ZeroDivisionError:
            # race condition between adding new objects and output
            if self.is_queue:
                return "Queue %s: not ready" % str(self.name)
            else:
                return "Process %s: not ready" % str(self.name)


class ProcessStatistics(object):

    def __init__(self, timeout=120):
        self.processes = {}
        self.queues = {}
        self.workers = {}
        self.timeout = timeout

    def __str__(self):
        return os.linesep.join([str(item)
                for item in self.processes.values() + self.queues.values()])

    def _refresh_workers(self):
        oldest_valid = time.time() - self.timeout
        for key in self.workers.keys():
            # be careful: maybe the workers dictionary changed in between
            try:
                timestamp = self.workers[key]
                if timestamp < oldest_valid:
                    del self.workers[key]
            except KeyError:
                pass

    def get_stats(self):
        return str(self)

    def add_transfer_time(self, name, amount):
        if not name in self.processes.keys():
            self.processes[name] = OneProcess(name)
        self.processes[name].transfer_count += 1
        self.processes[name].transfer_time += amount

    def add_process_time(self, name, amount):
        if not name in self.processes.keys():
            self.processes[name] = OneProcess(name)
        self.processes[name].process_count += 1
        self.processes[name].process_time += amount

    def add_queueing_time(self, name, amount):
        if not name in self.queues.keys():
            self.queues[name] = OneProcess(name, is_queue=True)
        self.queues[name].transfer_count += 1
        self.queues[name].transfer_time += amount

    def worker_notification(self, name):
        timestamp = time.time()
        self.workers[name] = timestamp

    def get_worker_statistics(self):
        self._refresh_workers()
        now = time.time()
        result = []
        # Cache the key list instead of iterating it - otherwise a
        # "RuntimeError: dictionary changed size during iteration" may occour.
        all_keys = self.workers.keys()
        for key in all_keys:
            try:
                one_process = self.processes[key]
                last_notification = int(now - self.workers[key])
                num_of_tasks = one_process.process_count
                process_time = one_process.process_time
                # avoid divide-by-zero
                avg_process_time = process_time / max(1, num_of_tasks)
                avg_transfer_time = one_process.transfer_time \
                        / max(1, num_of_tasks)
                result.append((key, last_notification, num_of_tasks,
                        process_time, avg_process_time, avg_transfer_time))
            except KeyError:
                # no data available yet or the item was removed meanwhile
                pass
        return result


class ProcessDataCache(object):

    def __init__(self, timeout=600):
        self.cache = {}
        self.timeout = timeout

    def _update_timestamp(self, name):
        if isinstance(name, ProcessDataCacheItemID):
            name = name.value
        now = time.time()
        try:
            self.cache[name][1] = now
        except KeyError:
            # the item was deleted meanwhile
            pass

    def expire_cache_items(self):
        expired = time.time() - self.timeout
        for key in self.cache.keys():
            try:
                if self.cache[key][1] < expired:
                    del self.cache[key]
            except KeyError:
                # ignore removed items
                pass

    def contains(self, name):
        if isinstance(name, ProcessDataCacheItemID):
            name = name.value
        self._update_timestamp(name)
        self.expire_cache_items()
        return name in self.cache.keys()

    def add(self, name, value):
        now = time.time()
        if isinstance(name, ProcessDataCacheItemID):
            name = name.value
        self.expire_cache_items()
        self.cache[name] = [value, now]

    def get(self, name):
        if isinstance(name, ProcessDataCacheItemID):
            name = name.value
        self._update_timestamp(name)
        self.expire_cache_items()
        return self.cache[name][0]


class ProcessDataCacheItemID(object):

    def __init__(self, value):
        self.value = value

