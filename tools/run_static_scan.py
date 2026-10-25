"""调用唯一雷达核心的有界静扫；默认只预览，不打开串口或发送车辆命令。"""
import argparse
import json
from pathlib import Path
import time
from urllib.error import HTTPError
from urllib.request import Request,urlopen


def read(port,path='/state'):
    with urlopen(f'http://127.0.0.1:{port}'+path,timeout=3) as response:
        return json.load(response)


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--zone',type=int,required=True,choices=(1,2))
    parser.add_argument('--duration',type=float,default=10)
    parser.add_argument('--request-id',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if not 2<=args.duration<=10 or not 1<=len(args.request_id)<=64:
        parser.error('窗口限定2～10秒，请求ID限定1～64字符')
    body=dict(request_id=args.request_id,zone=args.zone,duration_s=args.duration)
    if not args.execute:
        print(json.dumps(dict(preview=True,request=body,device_commands=0)))
        return
    args.output.mkdir(parents=True,exist_ok=False)
    board=read(8091)
    if not board.get('connected') or board['base']['fault'] or not board['base']['stopped']:
        raise RuntimeError('底盘状态不符；没有申请扫描')
    save(args.output/'board_before.json',board)
    save(args.output/'request.json',body)
    accepted=None
    for _ in range(10):
        request=Request('http://127.0.0.1:8093/scan',data=json.dumps(body).encode(),
            headers={'Content-Type':'application/json','X-Lidar-Workbench':'1'},method='POST')
        try:
            with urlopen(request,timeout=3) as response:accepted=json.load(response)
            break
        except HTTPError as exc:
            reason=exc.read().decode()
            if exc.code!=409 or '事务未结束' not in reason:
                save(args.output/'rejected.json',dict(code=exc.code,reason=reason))
                raise RuntimeError(reason) from exc
            time.sleep(.1)
    if not accepted:
        raise RuntimeError('只读事务尚未结束，未接纳扫描')
    save(args.output/'accepted.json',accepted)
    print(json.dumps(accepted),flush=True)
    rows=[];start=time.monotonic();previous=None
    while time.monotonic()-start < args.duration+10:
        state=read(8093);scan=state.get('scan')
        rows.append(dict(elapsed_s=time.monotonic()-start,state=state))
        if scan and scan['id']==accepted['id']:
            if previous!=scan['status']:
                print(json.dumps({k:scan.get(k) for k in ('status','cloud_frames','result_frames',
                    'matched_frames','stop_confirmed','errors')},ensure_ascii=True),flush=True)
                previous=scan['status']
            if scan['status'] in ('COMPLETE','FAILED'):
                break
        time.sleep(.15)
    save(args.output/'states.json',dict(accepted=accepted,samples=rows))
    save(args.output/'board_after.json',read(8091))
    result=rows[-1]['state'].get('scan')
    if not result or result['id']!=accepted['id'] or result['status'] not in ('COMPLETE','FAILED'):
        raise RuntimeError('未获得本次扫描终态，不自动重新发送扫描')
    save(args.output/'result.json',result)
    print(json.dumps({k:result.get(k) for k in ('id','status','cloud_frames','result_frames',
        'matched_frames','stop_confirmed','errors')},ensure_ascii=True),flush=True)
    if result['status']!='COMPLETE' or not result['stop_confirmed']:
        raise SystemExit(1)


if __name__=='__main__':main()
