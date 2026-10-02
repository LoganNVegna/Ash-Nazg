"""Exercise the shipped PowerShell downloader against a local paged HTTP server."""
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import hashlib, json, shutil, subprocess, threading

import tempfile
project=Path(__file__).resolve().parents[1]
workspace=tempfile.TemporaryDirectory(prefix='ashnazg-download-test-')
scratch=Path(workspace.name)
shutil.copyfile(project/'Download-Run.ps1',scratch/'Download-Run.ps1')
header=b'time_us,value\n'
rows=[f'{i*10000},{i%360}\n'.encode() for i in range(800)]
body=header+b''.join(rows)
state={'corrupt':False,'id':'abcdef11-800','build':'AshNazg competition 1.0'}
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def do_GET(self):
        parsed=urlparse(self.path);args=parse_qs(parsed.query)
        if parsed.path=='/export.json':
            data=json.dumps({'build':state['build'],'capture_id':state['id'],
                'rows':len(rows),'bytes':len(body),'sha256':hashlib.sha256(body).hexdigest()}).encode()
            content='application/json';digest=None
        elif parsed.path=='/summary.txt':data=b'frozen_capture=1\n';content='text/plain';digest=None
        elif parsed.path in ('/run.csv','/download-check.csv'):
            start=int(args['start'][0]);count=int(args['count'][0]);data=header+b''.join(rows[start:start+count])
            digest=hashlib.sha256(data).hexdigest();content='text/csv'
            if state['corrupt'] and start==25:data=data.replace(b',',b';',1)
        else:self.send_error(404);return
        self.send_response(200);self.send_header('Content-Type',content)
        self.send_header('Content-Length',str(len(data)))
        if digest:self.send_header('X-CSV-SHA256',digest)
        self.end_headers();self.wfile.write(data)
server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
def run(check=False):
    args=['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(scratch/'Download-Run.ps1'),
          '-RobotUrl',f'http://127.0.0.1:{server.server_port}']
    if check:args.append('-Check')
    return subprocess.run(args,capture_output=True,text=True,timeout=60)
try:
    result=run(True);assert result.returncode==0,result.stdout+result.stderr
    assert (scratch/'captures/check-abcdef11-800/AshNazg-download-check.csv').read_bytes()==body
    result=run();assert result.returncode==0,result.stdout+result.stderr
    folder=scratch/'captures/run-abcdef11-800'
    assert (folder/'AshNazg-competition.csv').read_bytes()==body
    assert 'frozen_capture=1' in (folder/'AshNazg-competition-summary.txt').read_text(encoding='utf-8-sig')
    state.update(build='AshNazg competition 1.1',id='abcdef33-800')
    result=run();assert result.returncode==0,result.stdout+result.stderr
    assert (scratch/'captures/run-abcdef33-800/AshNazg-competition.csv').read_bytes()==body
    state.update(build='AshNazg competition 1.2',id='abcdef55-800')
    result=run();assert result.returncode==0,result.stdout+result.stderr
    assert (scratch/'captures/run-abcdef55-800/AshNazg-competition.csv').read_bytes()==body
    state.update(build='AshNazg powered V5 baseline 1.1',id='abcdef44-800')
    result=run();assert result.returncode!=0 and 'Unsupported export format' in result.stderr,result.stdout+result.stderr
    assert not (scratch/'captures/run-abcdef44-800').exists()
    state['build']='AshNazg competition 1.1'
    state.update(corrupt=True,id='abcdef22-800')
    result=run();assert result.returncode!=0 and 'checksum mismatch' in result.stderr,result.stdout+result.stderr
    folder=scratch/'captures/run-abcdef22-800'
    assert (folder/'AshNazg-competition-summary.txt').is_file()
    assert (folder/'part-0000.csv').is_file()
    assert not (folder/'AshNazg-competition.csv').exists()
    print('PASS: competition 1.0/1.1/1.2 download 800 rows byte-for-byte; incompatible builds/corrupt pages rejected; summary and earlier pages preserved.')
finally:server.shutdown();server.server_close();thread.join();workspace.cleanup()
