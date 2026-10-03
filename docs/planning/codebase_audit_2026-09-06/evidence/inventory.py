"""Read-only source/config inventory; never imports repository applications."""
from pathlib import Path
import ast, collections, hashlib, json, os, subprocess
ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).parent
SKIP = {'.git', '__pycache__', '.pytest_cache', '.codebase-memory', 'node_modules', '.venv', 'venv'}
TEXT = {'.py', '.md', '.sh', '.yaml', '.yml', '.json', '.urdf', '.xml', '.launch', '.txt', '.js', '.html', '.css', '.rviz', '.cfg', '.ini', '.toml'}
tracked = set(subprocess.check_output(['git', 'ls-files'], cwd=ROOT, text=True).splitlines())
rows=[]
for directory, dirs, files in os.walk(ROOT):
    dirs[:] = sorted(d for d in dirs if d not in SKIP and Path(directory, d) != OUT.parent)
    for name in sorted(files):
        p=Path(directory, name); rel=p.relative_to(ROOT).as_posix()
        if p.suffix.lower() not in TEXT and name not in {'CMakeLists.txt', 'Dockerfile', '.gitignore'}: continue
        raw=p.read_bytes()
        try: src=raw.decode('utf-8-sig')
        except UnicodeDecodeError:
            try: src=raw.decode('utf-16')
            except UnicodeError: src=raw.decode('utf-8', errors='replace')
        row=dict(path=rel, tracked=rel in tracked, bytes=len(raw), lines=len(src.splitlines()), sha256=hashlib.sha256(raw).hexdigest())
        if p.suffix=='.py':
            try:
                tree=ast.parse(src, filename=rel)
                row['symbols']=[dict(name=n.name,line=n.lineno,kind=type(n).__name__) for n in ast.walk(tree) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef))]
                row['imports']=[ast.unparse(n) for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom))]
                row['calls']=sorted(set(ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)))
                try: ast.parse(src, feature_version=(3,8)); row['python38_syntax']='pass (not dependency/runtime validation)'
                except SyntaxError as e: row['python38_syntax']=str(e)
                row['ast']='pass'
            except SyntaxError as e: row['ast']=str(e)
        rows.append(row)
(OUT/'source_inventory.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
summary={'text_files':len(rows),'python_files':sum(x['path'].endswith('.py') for x in rows),'text_lines':sum(x['lines'] for x in rows),'extensions':dict(collections.Counter(Path(x['path']).suffix for x in rows)), 'parse_failures':[{k:x[k] for k in ('path','ast')} for x in rows if 'ast' in x and x['ast']!='pass'],'excluded_directories':sorted(SKIP),'audit_output_excluded':str(OUT.parent)}
(OUT/'inventory_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
for label,args in [('git_baseline', ['status','--short']),('git_history',['log','--all','--date=iso-strict','--format=%H %ad %s','--name-status']),('git_file_modes',['ls-files','--stage'])]:
    result=subprocess.run(['git']+args,cwd=ROOT,text=True,encoding='utf-8',errors='replace',capture_output=True)
    (OUT/(label+'.txt')).write_text(result.stdout+'\nSTDERR:\n'+result.stderr,encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
