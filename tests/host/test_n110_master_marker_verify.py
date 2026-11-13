"""A failed master layout stamp must leave startup incomplete and resumable."""
from unittest.mock import patch
import unittest

import test_n92_interrupted_migration as migration


class MarkerVerification(unittest.TestCase):
    def test_failed_or_silent_marker_write_keeps_checkpoint(self):
        original = migration.prelude
        def prelude():
            code = original()
            code = code.replace('static unsigned writes;', 'static unsigned writes; static int marker_fault;')
            code = code.replace('uint32_t old; memcpy(&old,', '''
 if(nv->index == 0 && nv->value_flt == 10003 && marker_fault) {
  if(marker_fault == 3) memcpy(eeprom, &nv->value_int, 2); // torn stamp without a reset
  return marker_fault == 1 ? 106 : 0; // explicit error or undetected write failure
 }
 uint32_t old; memcpy(&old,''')
            return code
        main = migration.MAIN.split('int main(void) {', 1)[0] + r'''
int main(void) {
 master_board(); assert(boot(0)); assert(config_loaded);
 memcpy(want_eeprom,eeprom,sizeof eeprom); memcpy(want_model,model,sizeof model);
 for(int mode=1;mode<=3;mode++) {
  master_board(); marker_fault=mode;
  assert(boot(0));
  assert(!config_loaded);
  index_t slot; uint8_t seq;
  assert(_master_checkpoint(&slot,&seq)>0);
  nvObj_t n={.index=0};read_persistent_value(&n);assert(!_nvm_layout_is_current(&n));
  // Another unsuccessful boot must preserve the recovery information too.
  assert(boot(0));assert(!config_loaded);assert(_master_checkpoint(&slot,&seq)>0);
  marker_fault=0; assert(boot(0));assert(config_loaded);same(mode,0);
 }
 return 0;
}
'''
        with patch.object(migration, 'prelude', prelude):
            for version in ('440.20f', '440.21f'):
                with self.subTest(version=version):
                    migration.InterruptedMasterMigration().build(main, version)
