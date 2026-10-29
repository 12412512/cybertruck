"""Read frozen executor and per-wheel diagnostics; attach, no halt/reset/write."""
import argparse,hashlib,json,struct,time,sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
sys.path.insert(0,str(Path.cwd()/'tools'))

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--flash-dir',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--frequency',type=int,default=500000)
args=parser.parse_args()
root=Path.cwd();folder=args.flash_dir
manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
assert manifest['verified'] and manifest['resumed_after_verify']
assert hashlib.sha256((folder/'symbols/Car.hex').read_bytes()).hexdigest()==manifest['image_sha256']
names={'executor':'Exec','bus_diag':'BusDiag','can_diag':'CanDiag','wheels':'BusWheel','micro_trace':'MicroTrace'}
with (folder/'symbols/Car.axf').open('rb') as stream:
 elf=ELFFile(stream);table=elf.get_section_by_name('.symtab');types={}
 for cu in elf.get_dwarf_info().iter_CUs():
  for die in cu.iter_DIEs():
   if die.tag=='DW_TAG_typedef' and 'DW_AT_name' in die.attributes:
    n=die.attributes['DW_AT_name'].value.decode()
    if n in names.values():types[n]=die
 def size_of(die):
  if die.tag=='DW_TAG_pointer_type':return die.attributes['DW_AT_byte_size'].value if 'DW_AT_byte_size' in die.attributes else 4
  if 'DW_AT_byte_size' in die.attributes:return die.attributes['DW_AT_byte_size'].value
  if die.tag=='DW_TAG_array_type':
   count=1
   for bound in die.iter_children():
    if bound.tag=='DW_TAG_subrange_type':
     count*=bound.attributes['DW_AT_count'].value if 'DW_AT_count' in bound.attributes else bound.attributes['DW_AT_upper_bound'].value+1
   return count*size_of(die.get_DIE_from_attribute('DW_AT_type'))
  return size_of(die.get_DIE_from_attribute('DW_AT_type'))
 def decode(die,data):
  if die.tag in ('DW_TAG_typedef','DW_TAG_const_type','DW_TAG_volatile_type'):
   return decode(die.get_DIE_from_attribute('DW_AT_type'),data)
  if die.tag in ('DW_TAG_structure_type','DW_TAG_union_type'):
   result={}
   for m in die.iter_children():
    if m.tag!='DW_TAG_member' or 'DW_AT_name' not in m.attributes:continue
    off=m.attributes.get('DW_AT_data_member_location');off=off.value if off else 0
    typ=m.get_DIE_from_attribute('DW_AT_type');size=size_of(typ)
    result[m.attributes['DW_AT_name'].value.decode()]=decode(typ,data[off:off+size])
   return result
  if die.tag=='DW_TAG_array_type':
   item=die.get_DIE_from_attribute('DW_AT_type');size=size_of(item)
   return [decode(item,data[i:i+size]) for i in range(0,len(data),size)]
  if die.tag=='DW_TAG_base_type' and die.attributes.get('DW_AT_encoding'):
   enc=die.attributes['DW_AT_encoding'].value
   if enc==4:return struct.unpack('<f' if len(data)==4 else '<d',data)[0]
   return int.from_bytes(data,'little',signed=enc in (5,6))
  return int.from_bytes(data,'little')
 frequency=args.frequency
 options=dict(pack=manifest['pack'],target_override='stm32f407zg',frequency=frequency,
  connect_mode='attach',halt_on_connect=False,resume_on_disconnect=False,auto_unlock=False,config_file=None)
 session=ConnectHelper.session_with_chosen_probe(unique_id=manifest['probe'],blocking=False,options=options)
 assert session is not None,'ST-Link unavailable; unchanged'
 record=dict(writes=0,reset=False,halt=False,frequency=frequency,image_sha256=manifest['image_sha256'])
 with session:
  target=session.target;assert target.get_state()==Target.State.RUNNING
  assert bytes(target.read_memory_block8(0x1FFF7A10,12)).hex()==manifest['uid']
  for name,typename in names.items():
   symbol=max(table.get_symbol_by_name(name),key=lambda s:s['st_size']);address=symbol['st_value'];size=symbol['st_size']
   raw=bytes(target.read_memory_block8(address,size));typ=types[typename]
   stride=size_of(typ)
   value=[decode(typ,raw[i:i+stride]) for i in range(0,size,stride)] if name=='wheels' else decode(typ,raw)
   record[name]=dict(address=address,raw_hex=raw.hex(),value=value)
  assert target.get_state()==Target.State.RUNNING
 path=args.output
 path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(json.dumps(record,indent=2,allow_nan=False),encoding='utf-8')
 trace=record['micro_trace']['value']
 print(json.dumps(dict(path=str(path),action=trace['action'],count=trace['count'],
    frozen=bool(trace['frozen']),exec_fault=trace['exec_fault'],track_fault=trace['track_fault'],
    can_errors=record['can_diag']['value']['errors'],reset=False,halt=False,writes=0)))
