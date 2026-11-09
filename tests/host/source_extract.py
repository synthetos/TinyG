"""Extract complete C function definitions for host fixtures."""
import re


def function(source, signature):
    match = re.search(signature + r'[^\{]*\{', source)
    assert match, signature
    depth = 1
    tokens = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/|[{}]', re.S)
    for token in tokens.finditer(source, match.end()):
        depth += (token.group() == '{') - (token.group() == '}')
        if depth == 0:
            return source[match.start():token.end()]
    raise AssertionError(signature)



def region(source, start, last_signature):
    """The source from the first match of `start` through the end of the function `last_signature`,
    for static state that lives between functions."""
    begin = re.search(start, source)
    assert begin, start
    last = function(source[begin.start():], last_signature)
    end = source.index(last, begin.start()) + len(last)
    return source[begin.start():end]


# runtime.h stand-ins for tests that assemble firmware functions which clear the quiescence token or
# reset the runtime state; the rt family itself is exercised in test_runtime_rt.py.
RUNTIME_STUBS = r'''
#ifndef RUNTIME_STUBS
#define RUNTIME_STUBS
static struct { bool allowed, fl_taken; } rt __attribute__((unused));
static inline __attribute__((unused)) void rt_qt_invalidate(void) {}
static inline __attribute__((unused)) void rt_init(void) {}
static inline __attribute__((unused)) void rt_job_end(void) {}
#endif
'''

# Orthogonal override hooks default to the pre-feature state in legacy extracted-function fixtures.
# test_override_* links the actual module and does not use these stubs.
OVERRIDE_STUBS = r'''
#ifndef OVERRIDE_FIXTURE_STUBS
#define OVERRIDE_FIXTURE_STUBS
#include <stdint.h>
#include <stdbool.h>
static inline __attribute__((unused)) void ov_init(void) {}
static inline __attribute__((unused)) void ov_cancel(void) {}
static inline __attribute__((unused)) bool _usb_override_byte(uint8_t c,uint8_t status) {(void)c;(void)status;return false;}
#endif
'''
RUNTIME_STUBS += OVERRIDE_STUBS
