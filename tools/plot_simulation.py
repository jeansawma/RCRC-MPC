"""Generate reproducible response curves with the actual firmware C controllers."""
import ctypes
import json
import pathlib
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def main():
    lib=ctypes.CDLL(str(ROOT/"build/libsimulation.so"))
    lib.sim_create.restype=ctypes.c_void_p
    lib.sim_destroy.argtypes=[ctypes.c_void_p]
    lib.sim_sampling_us.argtypes=[ctypes.c_void_p]
    lib.sim_sampling_us.restype=ctypes.c_uint32
    lib.sim_command.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_void_p,ctypes.c_size_t]
    lib.sim_advance.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_size_t]
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,layout="constrained")
    names=["PI (PID with Kd=0)","Finite control set MPC","Continuous control set MPC — QP"]
    for mode in range(3):
        handle=lib.sim_create(); assert handle
        out=ctypes.create_string_buffer(16384)
        for command in [f"SET mode {mode}","ARM"]:
            assert lib.sim_command(handle,command.encode(),out,len(out))
            assert json.loads(out.value)["type"]=="ack",out.value
        records=[]
        sample_us=lib.sim_sampling_us(handle)
        total_steps=int(5e6/sample_us)
        first_step=int(1.5e6/sample_us);second_step=int(3.5e6/sample_us)
        for step in range(total_steps):
            if step in [first_step,second_step]:
                command=b"SET reference 10" if step==first_step else b"SET reference 4"
                lib.sim_command(handle,command,out,len(out))
                assert json.loads(out.value)["type"]=="ack"
            lib.sim_advance(handle,1,out,len(out))
            record=json.loads(out.value)
            assert record["armed"] and not record["fault"],record
            records.append(record)
        lib.sim_destroy(handle)
        t=[r["t_us"]/1e6 for r in records]
        left,right=axes[mode]
        left.plot(t,[r["reference"] for r in records],"--",color="#7b8498",label="Reference")
        left.plot(t,[r["v1"] for r in records],linewidth=1,color="#f6a21a",label="First capacitor")
        left.plot(t,[r["v2"] for r in records],linewidth=1.5,color="#147d92",label="Output")
        left.set_title(names[mode],loc="left",weight="bold");left.set_ylabel("Voltage (V)")
        right.plot(t,[r["duty"] for r in records],linewidth=.65,color="#6862b4")
        right.set_ylabel("Switch state" if mode==1 else "PWM duty");right.set_ylim(-.05,1.05)
        for ax in (left,right):ax.grid(alpha=.2);ax.spines[["top","right"]].set_visible(False)
        if mode==0:left.legend(loc="lower right",ncol=3,fontsize=8)
    for ax in axes[-1]:ax.set_xlabel("Time (s)")
    fig.suptitle("RCRC software simulation · 24 V bus · 470 Ω / 100 µF per stage\nSame C control code as firmware; ideal switching model, no hardware measurements",fontsize=13)
    target=ROOT/"docs/simulation.png";fig.savefig(target,dpi=160)
    print(target)

if __name__=="__main__":main()
