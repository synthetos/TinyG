"""The boot settings fallback of config.c (N61) as C text for host tests: the report lists and their functions."""
from source_extract import function

FUNCTIONS = ("_report_list_add", "_report_list_send", "_load_config_default", "_load_config_fallback",
             "config_report_load_failure")


def fallback_source(config_c, names=FUNCTIONS):
    """The report-list type and state, then the named functions, extracted from config.c's text."""
    src = config_c.replace("\r\n", "\n")
    state = src[src.index("#define CONFIG_REPORT_NAMES"):src.index("static void _report_list_add")]
    code = state + r"""
// Layout-2 row migration has separate interrupted-write coverage.
static inline bool _layout2_journal_ready(void) { return false; }
static inline void _migrate_layout2(nvObj_t *nv) { (void)nv; }
"""
    for name in names:
        code += function(src, r"(?:static )?(?:void|stat_t) " + name + r"\([^;{}]*\)(?=\s*\{)") + "\n"
    return code


# For tests that model a setter refusing its own default: GET_TABLE_WORD(set) yields a setter that refuses.
REFUSING_SETTER = r'''
typedef stat_t (*fptrCmd)(nvObj_t *nv);
static stat_t n61_refusing_setter(nvObj_t *nv) { (void)nv; return 106; }
#define GET_TABLE_WORD(a) n61_table_##a
#define n61_table_set n61_refusing_setter
'''
