"""Regenerate the narrowly scoped 449 display metadata from an unchanged source tree."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def extract(source):
    entries={}
    for token in ('bjd','cjd'):
        matches=list(re.finditer(r'^.*\{\s*"[^"]+"\s*,\s*"'+token+r'"\s*,\s*[^,]+,\s*(\d+)\s*,.*$',source,re.M))
        if len(matches)!=1:raise ValueError('expected exactly one cfgArray row for '+token)
        m=matches[0]
        entries[token]=dict(precision=int(m[1]),line=source[:m.start()].count('\n')+1,row=m[0].strip())
    return entries


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();rel='firmware/tinyg/config_app.c';raw=(args.source/rel).read_bytes()
    committed=subprocess.check_output(['git','-C',str(args.source),'show','HEAD:'+rel])
    if raw!=committed:ap.error('cfgArray source differs from its recorded commit')
    header=(args.source/'firmware/tinyg/tinyg.h').read_text()
    if not re.search(r'#define\s+TINYG_FIRMWARE_BUILD\s+449\.01\b',header):ap.error('requires449.01 source')
    result=dict(commit=subprocess.check_output(['git','-C',str(args.source),'rev-parse','HEAD'],text=True).strip(),
                file=rel,sha256=hashlib.sha256(raw).hexdigest(),fb=449.01,tokens=extract(raw.decode()))
    args.out.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
