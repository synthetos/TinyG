"""A/B settings contract: one immutable file, explicit overrides, actual readback."""
import copy
import hashlib
import json
import math
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
from . import snapshot


LEGACY_PRECISION = json.loads((Path(__file__).parent/'data/perf-449-print-precision.json').read_text())


def printed_expected(tg,token,value):
    # Only these source-backed legacy quirks; never widen ordinary float tolerance.
    if float(tg.identity['fb']) != LEGACY_PRECISION['fb'] or token not in LEGACY_PRECISION['tokens']:
        return None
    if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value):return None
    precision=LEGACY_PRECISION['tokens'][token]['precision']
    return float(Decimal(str(value)).quantize(Decimal(10)**-precision,rounding=ROUND_HALF_UP))


def new_control(token):
    return token.startswith('rt') or token in ('spph','spde','coph')


def load(path):
    if not path:raise ValueError('PERF_SNAPSHOT (one fixed bench snapshot) is required')
    path=Path(path).expanduser().resolve();raw=path.read_bytes();data=json.loads(raw)
    values=data.get('values')
    if not isinstance(values,dict) or not values:raise ValueError('fixed snapshot has no settings')
    # Do not accept a partial profile in place of a saved bench configuration.
    required={f'{m}{k}' for m in range(1,5) for k in ('ma','sa','tr','mi','po')}
    if not required<=values.keys():raise ValueError('fixed snapshot is missing motor geometry/map')
    return data,dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())


def for_board(data,fb):
    result=copy.deepcopy(data)
    result['values']={k:v for k,v in data['values'].items()
                      if k not in snapshot.NEVER_WRITE and not snapshot.INTERNAL.match(k) and k!='_hld'
                      and k not in snapshot.USER_DATA and not (fb<450 and new_control(k))}
    return result


def verify(tg,data,*,overrides=None):
    expected_id=data.get('identity',{}).get('id')
    if expected_id and expected_id!=tg.identity.get('id'):raise ValueError('fixed snapshot belongs to another board')
    expected=for_board(data,float(tg.identity['fb']))['values']
    overrides=overrides or {}
    expected.update({k:v for k,v in overrides.items() if k in expected})
    actual={};bad={};reduced=[]
    for token,want in expected.items():
        try:
            actual[token]=tg.get(token)
            if not tg.values_equal(token,actual[token],want):
                printed=printed_expected(tg,token,want)
                if printed is not None and actual[token]==printed:
                    reduced.append(dict(token=token,expected=want,actual=actual[token],printed_expected=printed,
                                        **LEGACY_PRECISION['tokens'][token]))
                else:bad[token]=dict(expected=want,actual=actual[token])
        except Exception as e:bad[token]=dict(expected=want,error=str(e))
    fields=tg.get('sr')
    fields=list(fields) if isinstance(fields,dict) else fields
    if data.get('sr') is not None and fields!=data['sr']:bad['sr']=dict(expected=data['sr'],actual=fields)
    if bad:raise ValueError('fixed snapshot mismatch: '+json.dumps(bad,sort_keys=True))
    return dict(ok=True,values=actual,sr=fields,overrides=overrides,
                not_restorable=snapshot.not_restorable(data['values']),reduced_precision=reduced,
                precision_source={k:v for k,v in LEGACY_PRECISION.items() if k!='tokens'} if reduced else None)


def restore_verified(tg,data):
    expected_id=data.get('identity',{}).get('id')
    if expected_id and expected_id!=tg.identity.get('id'):raise ValueError('fixed snapshot belongs to another board')
    def equal(token,actual,want):
        printed=printed_expected(tg,token,want)
        return tg.values_equal(token,actual,want) or (printed is not None and actual==printed)
    force=[k for k in data['values'] if printed_expected(tg,k,data['values'][k]) is not None]
    # A rounded zero cannot tell default0 from intended0.05. Always SET the real
    # requested value first; only the post-write readback gets the display check.
    result=snapshot.restore(tg,for_board(data,float(tg.identity['fb'])),readback_equal=equal,force_tokens=force)
    if result.get('failed'):raise ValueError('fixed snapshot restore failed: '+str(result['failed']))
    return dict(restore=result,verification=verify(tg,data))
