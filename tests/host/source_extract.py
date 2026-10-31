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

