"""Validate report links, evidence consistency and preservation of source files."""
from pathlib import Path
import hashlib,json,re,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).parent
rows=json.loads((OUT/'source_inventory.json').read_text(encoding='utf-8'))
modified=[]
for row in rows:
 p=ROOT/row['path']
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:modified.append(row['path'])
broken=[]
reports=list(OUT.parent.glob('*.md'))
for p in reports:
 for dest in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
  if '://' in dest or dest.startswith('#'):continue
  if not (p.parent/dest.split('#')[0]).exists():broken.append({'report':p.name,'target':dest})
urdfs=[]
for row in rows:
 if row['path'].startswith('.claude/') or not row['path'].endswith('.urdf'):continue
 p=ROOT/row['path'];tree=ET.parse(p).getroot()
 meshes=sorted(set(n.attrib['filename'] for n in tree.findall('.//mesh')))
 urdfs.append({'path':row['path'],'links':len(tree.findall('link')),'joints':len(tree.findall('joint')),'missing_meshes':[m for m in meshes if '://' not in m and not (p.parent/m).exists()],'sha256':row['sha256'],'fixed_joints':[{'name':j.attrib['name'],'parent':j.find('parent').attrib,'child':j.find('child').attrib,'origin':j.find('origin').attrib if j.find('origin') is not None else {}} for j in tree.findall('joint') if j.attrib['type']=='fixed']})
models=[]
for version in ('v21','v23'):
 p=ROOT/'grasp'/version/'manifest.json';m=json.loads(p.read_text(encoding='utf-8'))
 for key in ('model','vecnormalize'):
  item=m['artifacts'][key];f=p.parent/item['file'];actual=hashlib.sha256(f.read_bytes()).hexdigest()
  models.append({'version':version,'artifact':key,'path':str(f.relative_to(ROOT)),'matches_manifest':actual==item['sha256'],'sha256':actual})
tests=json.loads((OUT/'offline_test_results.json').read_text(encoding='utf-8'))
result={'reports':sorted(p.name for p in reports),'source_files_changed_since_inventory':modified,'broken_local_links':broken,'offline_commands':len(tests),'offline_failed':[x for x in tests if x['exit_code']!=0],'urdf_validation':urdfs,'model_hash_validation':models}
(OUT/'audit_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
assert len(reports)==7 and not modified and not broken
assert all(x['matches_manifest'] for x in models)
