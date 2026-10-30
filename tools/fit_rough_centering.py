"""离线拟合圆环小位移矩阵；不连接相机、串口或网络，不自动批准实物验收。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi'))
from robot.vision.centering_fit import fit_records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--ids',required=True,help='同一观察位正反X/Y记录ID，用逗号分开')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--calibration-id',type=int,default=1)
    p.add_argument('--radius-px',type=float,default=12)
    a=p.parse_args()
    if a.output.exists():p.error('输出已存在，请使用新文件名，不覆盖旧标定')
    try:
        value=fit_records(json.loads(a.samples.read_text(encoding='utf-8')),
            [int(i) for i in a.ids.split(',')],a.calibration_id,a.radius_px)
    except (ValueError,OSError,KeyError,TypeError) as exc:p.error(str(exc))
    a.output.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(saved=str(a.output),sha256=value['sha256'],physical_reviewed=False,automatic_allowed=False)))


if __name__=='__main__':main()
