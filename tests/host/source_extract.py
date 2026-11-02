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
#endif
'''
