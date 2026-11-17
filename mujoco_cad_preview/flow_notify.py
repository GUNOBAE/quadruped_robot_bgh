"""Local Windows completion alarm. No cloud/phone notification or scheduler.
Call in a SEPARATE process after saving final status and releasing compute locks.
"""
from __future__ import annotations
import argparse,json,os,time
from pathlib import Path

def notify(run,status,test=False,silent=False):
    run=Path(run);run.mkdir(parents=True,exist_ok=True)
    path=run/'notification.json'
    record={'status':status,'local_pc_only':True,'requested_at':time.time(),'test':test,'sound':'not_attempted','dialog':'not_attempted'}
    def save():path.write_text(json.dumps(record,indent=2,ensure_ascii=False),encoding='utf-8')
    save()
    good=status=='CONFIRMED_FLOW_SAVED'
    title='BGH 보행 실험: '+('알림 시험' if test else '개선 후보 확인' if good else '종료 · 결과 확인')
    message=(('완료 알림이 정상적으로 표시됩니다.' if test else '보행 실험이 종료되었습니다.\n상태: '+status)
             +'\n\n결과 폴더:\n'+str(run)+'\n\nSUMMARY.md에서 결과를 확인하세요.\n이 알림을 닫아도 저장된 결과는 유지됩니다.')
    if os.name!='nt' or silent:
        record['dialog']='skipped_non_windows_or_silent';record['sound']='skipped_non_windows_or_silent';save();return record
    try:
        import winsound
        for frequency in (660,880,1100):
            winsound.Beep(frequency,220)
        record['sound']='requested' # OS/device mute can still prevent hearing it.
    except Exception as exc:record['sound']='error: '+repr(exc)
    save()
    try:
        import ctypes
        from ctypes import wintypes as W
        fn=ctypes.WinDLL('user32',use_last_error=True).MessageBoxW
        fn.argtypes=[W.HWND,W.LPCWSTR,W.LPCWSTR,W.UINT];fn.restype=ctypes.c_int
        record['dialog']='requested';save()
        result=fn(None,message,title,0x00040000|0x00010000|0x00000040)
        record['dialog']='acknowledged' if result else 'failed'
    except Exception as exc:record['dialog']='error: '+repr(exc)
    save();return record

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--status',default='ALERT_TEST')
    p.add_argument('--test',action='store_true');p.add_argument('--silent',action='store_true');a=p.parse_args()
    notify(a.run,a.status,a.test,a.silent)
