"""Build reviewed minimal deltas for deployed v23; refuse unknown source.

Writes candidates only. Deployment must separately verify original hashes,
back up files and stop the sole Serial owner before replacing/restarting it.
"""
import ast
from pathlib import Path


def replace_once(source,before,after):
    if source.count(before)!=1:
        raise ValueError('deployment source changed or patch already applied: '+before[:60])
    return source.replace(before,after,1)


def patch_service(source):
    if 'health_server.start()' in source or 'from legacy_health_compat import' in source:
        raise ValueError('independent health already installed')
    tree=ast.parse(source)
    methods=[node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name=='cmd_health']
    if len(methods)!=1:
        raise ValueError('expected one cmd_health')
    method=methods[0]
    lines=source.splitlines(keepends=True)
    body='''    def cmd_health(self) -> dict:
        """Independent cached snapshot; never performs a servo transaction."""
        from legacy_health_compat import health_snapshot
        return health_snapshot(self,rss_mb())
'''
    source=''.join(lines[:method.lineno-1])+body+''.join(lines[method.end_lineno:])
    source=replace_once(source,'        srv.bind(self.sock_path)\n',
        '''        srv.bind(self.sock_path)
        from health_server import HealthServer
        health_path='/tmp/grasp_health.sock' if self.sock_path=='/tmp/grasp_service.sock' else self.sock_path+'.health'
        health_server=HealthServer(self.cmd_health,health_path)
        health_server.start()
''')
    source=replace_once(source,'            srv.close()\n','            health_server.close()\n            srv.close()\n')
    ast.parse(source)
    return source


def patch_vision(source):
    if 'from frame_health import write_frame_health' in source:
        raise ValueError('frame heartbeat already installed')
    source=replace_once(source,'            dt_infer = time.monotonic() - t_infer\n',
        '''            dt_infer = time.monotonic() - t_infer
            from frame_health import write_frame_health
            write_frame_health(os.environ.get('GRASP_VISION_HEALTH_FILE','/tmp/x3plus_vision_health.json'),frame.shape,dt_infer)
''')
    ast.parse(source)
    return source


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    root,out=Path(args.source),Path(args.output)
    out.mkdir(parents=True,exist_ok=True)
    for name,patch in [('grasp_service.py',patch_service),('vision_grasp_bridge.py',patch_vision)]:
        (out/name).write_text(patch((root/name).read_text(encoding='utf-8')),encoding='utf-8')


if __name__=='__main__':
    main()
