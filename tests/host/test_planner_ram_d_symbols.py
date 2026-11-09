"""D's typed payload keeps existing debugger paths through an anonymous union."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]


class Die:
    def __init__(self, tag, attrs=None, children=(), target=None):
        self.tag=tag
        self.attributes={k:SimpleNamespace(value=v) for k,v in (attrs or {}).items()}
        self.children=children
        self.target=target
    def iter_children(self): return iter(self.children)
    def get_DIE_from_attribute(self, attr):
        assert attr=='DW_AT_type'
        return self.target


def member(name, offset, target, extra=None):
    attrs={'DW_AT_data_member_location':offset,'DW_AT_type':1}
    if name: attrs['DW_AT_name']=name.encode()
    attrs.update(extra or {})
    return Die('DW_TAG_member',attrs,target=target)


class AnonymousPayloadSymbols(unittest.TestCase):
    def test_existing_member_array_and_bitfield_paths_cross_the_payload(self):
        tree=ast.parse((ROOT/'tests/hil/tinyg_hil/ice.py').read_text())
        original=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Symbols')
        selected=[n for n in original.body if isinstance(n,ast.FunctionDef) and n.name in ('_strip','_members','resolve')]
        cls=ast.ClassDef(name='Symbols',bases=[],keywords=[],body=selected,decorator_list=[])
        ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'symbols','exec'),ns)
        symbols=ns['Symbols']()
        flt=Die('DW_TAG_base_type',{'DW_AT_byte_size':4,'DW_AT_encoding':4})
        uint=Die('DW_TAG_base_type',{'DW_AT_byte_size':4,'DW_AT_encoding':7})
        byte=Die('DW_TAG_base_type',{'DW_AT_byte_size':1,'DW_AT_encoding':7})
        vector=Die('DW_TAG_array_type',target=flt)
        gm=Die('DW_TAG_structure_type',{'DW_AT_byte_size':64},[
            member('linenum',0,uint),member('target',4,vector),
            member('motion_mode',56,byte,{'DW_AT_bit_size':3,'DW_AT_byte_size':1,'DW_AT_bit_offset':5})])
        payload=Die('DW_TAG_union_type',{'DW_AT_byte_size':64},[member('gm',0,gm),member('dwell_seconds',0,flt)])
        block=Die('DW_TAG_structure_type',{'DW_AT_byte_size':163},[
            member(None,97,payload),member(None,96,byte),member('fl_tags',161,byte)])
        buffers=Die('DW_TAG_array_type',target=block)
        pool=Die('DW_TAG_structure_type',children=[member('bf',12,buffers)])
        symbols.sym={'mb':(0x800000+1000,12+32*163)}
        symbols.sram_symbol_base=0x800000
        symbols.types={'mb':pool}
        self.assertEqual(symbols.resolve('mb.bf[1].gm.linenum'),(1272,4,'uint'))
        self.assertEqual(symbols.resolve('mb.bf[1].gm.target[3]'),(1288,4,'float'))
        self.assertEqual(symbols.resolve('mb.bf[1].dwell_seconds'),(1272,4,'float'))
        self.assertEqual(symbols.resolve('mb.bf[1].gm.motion_mode'),(1328,1,'bits:0:3'))
        self.assertEqual(symbols.resolve('mb.bf[1].fl_tags'),(1336,1,'uint'))
        with self.assertRaises(KeyError): symbols.resolve('mb.bf[1].unknown')


if __name__=='__main__': unittest.main()
