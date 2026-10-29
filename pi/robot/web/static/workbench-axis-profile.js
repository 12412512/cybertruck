/* 依据Emm手册第57页计算理论曲线，纯计算，不调用任何设备。 */
export function axisProfile(axis,rpm,accel=1,distanceMm=null) {
    if(!['lift','reach'].includes(axis)||!Number.isInteger(rpm)||rpm<1||rpm>3000||
        !Number.isInteger(accel)||accel<0||accel>255)throw Error('转速1～3000RPM；加速度档位0～255整数');
    const mmPerRev=axis==='lift'?40.98:124,rate=accel===0?null:1/((256-accel)*.00005);
    const result={rpm,accel,rateRpmS:rate,ceilingMmS:rpm*mmPerRev/60,peakRpm:null,seconds:null};
    if(distanceMm===null)return result;
    if(!Number.isFinite(distanceMm)||distanceMm<0)throw Error('位移必须为非负毫米');
    const turns=distanceMm/mmPerRev;
    result.peakRpm=turns===0?0:rate===null?rpm:Math.min(rpm,Math.sqrt(turns*60*rate));
    result.seconds=turns===0?0:rate===null?turns*60/rpm:
        turns<rpm*rpm/(60*rate)?2*Math.sqrt(turns*60/rate):turns*60/rpm+rpm/rate;
    return result;
}

export function axisRateText(axis,rpm,accel,distanceMm=null) {
    try {
        const p=axisProfile(axis,rpm,accel,distanceMm);
        return `匀速换算上限${p.ceilingMmS.toFixed(1)}mm/s；${p.rateRpmS===null?'acc=0直接启动':`加速度约${p.rateRpmS.toFixed(1)}RPM/s`}。`+
            (p.seconds===null?'取得有效位置后显示本次理论曲线。':
             `本次理论峰值约${p.peakRpm.toFixed(1)}RPM，电机曲线${p.seconds.toFixed(2)}s（不含通信和到位等待）。`);
    }catch(error){return error.message;}
}
