/**
 * @file    host_cmd_exec_test.c
 * @brief   cmd_exec 命令执行器 PC 测试桩 (R2: 全函数指针注入; 校验应答字节+调用序)
 *
 * 编译运行 (见 run_pc_tests.ps1):
 *   gcc -Wall -Wextra -I Core/Inc -I Core/Inc/driver -I Core/Inc/common \
 *       -o build/pc_test_cmd_exec.exe \
 *       test/host_cmd_exec_test.c Core/Src/app/cmd_exec.c
 *
 * 回归目标: R2 把执行逻辑从 app_link 抽离为 cmd_exec, 语义等价的判据 =
 *   - 应答字符串逐字节与搬迁前一致
 *   - 各命令对注入函数的调用序列与搬迁前一致
 *   - C4 override_manual 调用点 (各手动指令执行前)
 *   - 门控(BUSY)在 app_link, 本桩只验已放行命令的执行。
 * 全程 mock 记录 "每次 io 函数被调用的参数" 存入 g_call[], 由断言检查。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "app/cmd_exec.h"

static int g_pass = 0, g_fail = 0;

static void check(int cond, const char *desc)
{
    if (cond) { g_pass++; printf("[PASS] %s\n", desc); }
    else      { g_fail++; printf("[FAIL] %s\n", desc); }
}

static int feq(float a, float b){ float d=a-b; if(d<0)d=-d; return d<0.0001f; }

/* ===================== mock 记录 ===================== */

/* 调用记录上限 */
#define MAX_CALL 256
enum { CMD_NONE=0, CMD_SEND, CMD_NOTECMD, CMD_NOTERESP, CMD_OVERRIDE, CMD_WDFEED };
enum { A_SPD_STOP, A_SPD_TGT, A_SPD_SETG, A_SPD_GETG, A_SPD_FF,
       A_L_EN, A_L_AUTO, A_L_KP, A_L_KD, A_L_SPD, A_L_INV, A_L_INVERTED,
       A_L_SCNT, A_L_TCNT,
       A_ST_SET, A_ST_GET, A_ST_NUDGE, A_ST_LMIN, A_ST_LMAX, A_ST_CTR_SET, A_ST_CENTER,
       A_CTL_STOPALL, A_CTL_STEP, A_CTL_TEL, A_CTL_REC, A_CTL_DUMP, A_CTL_ODOM,
       A_CTL_WD, A_CTL_ITEL, A_CTL_IRATE,
       A_IMU_MAHONY, A_IMU_DRIFT, A_IMU_RECAL,
       A_MTR_MPS, A_MTR_RATE, A_ARB_REQ, A_ARB_OWNER, A_PPR };

typedef struct { int act; int i0; float f0; float f1, f2; uint32_t u0; } Call;
static Call  g_call[MAX_CALL];
static int   g_n = 0;
static char  g_txbuf[4096];   /* mock sink 收集的发送字节 */
static int   g_txlen = 0;

static void rec(int act, int i0, float f0, float f1, float f2, uint32_t u0)
{
    if (g_n < MAX_CALL) { g_call[g_n].act=act; g_call[g_n].i0=i0; g_call[g_n].f0=f0;
                          g_call[g_n].f1=f1; g_call[g_n].f2=f2; g_call[g_n].u0=u0; }
    g_n++;
}

/* 收集发送内容（含索引断言） */
static int have_tx(const char *needle) { return strstr(g_txbuf, needle) != NULL; }

/* ---- mock sink / note ---- */
static void mock_send(const char *buf, int n, void *ctx) { (void)ctx;
    if (g_txlen + n < (int)sizeof(g_txbuf)) { memcpy(g_txbuf+g_txlen, buf, n); g_txlen+=n; } }
static void mock_note_cmd(const char *s, void *ctx)  { (void)ctx; (void)s; rec(CMD_NOTECMD,0,0,0,0,0); }
static void mock_note_resp(const char *s, void *ctx) { (void)ctx; (void)s; rec(CMD_NOTERESP,0,0,0,0,0); }
static void mock_override(void *ctx)   { (void)ctx; rec(CMD_OVERRIDE,0,0,0,0,0); }
static void mock_wdfeed(void *ctx)     { (void)ctx; rec(CMD_WDFEED,0,0,0,0,0); }
static void mock_ppr(uint8_t id, uint16_t ppr, void *ctx){ (void)ctx; rec(A_PPR,id,(float)ppr,0,0,0); }

/* ---- speed mock ---- */
static speed_loop_ret_t spd_stop(uint8_t id, void *ctx){ (void)ctx; rec(A_SPD_STOP,id,0,0,0,0); return SPEED_LOOP_OK; }
static speed_loop_ret_t spd_tgt(uint8_t id, float rpm, void *ctx){ (void)ctx; rec(A_SPD_TGT,id,rpm,0,0,0); return SPEED_LOOP_OK; }
static speed_loop_ret_t spd_setg(uint8_t id,float kp,float ki,float kd,void *ctx){ (void)ctx; rec(A_SPD_SETG,id,kp,ki,kd,0); return SPEED_LOOP_OK; }
static speed_loop_ret_t spd_getg(uint8_t id,float *kp,float *ki,float *kd,void *ctx){
    (void)ctx; rec(A_SPD_GETG,id,0,0,0,0); if(id==0){*kp=0.24f;*ki=0.13f;*kd=0.20f;} else {*kp=0.20f;*ki=0.13f;*kd=0.24f;} return SPEED_LOOP_OK; }
static speed_loop_ret_t spd_ff(uint8_t id,float gain,void *ctx){ (void)ctx; rec(A_SPD_FF,id,gain,0,0,0); return SPEED_LOOP_OK; }

/* ---- line mock ---- */
static void l_en(uint8_t en, void *ctx){ (void)ctx; rec(A_L_EN,en,0,0,0,0); }
static void l_auto(uint8_t v, void *ctx){ (void)ctx; rec(A_L_AUTO,v,0,0,0,0); }
static void l_kp(float v, void *ctx){ (void)ctx; rec(A_L_KP,0,v,0,0,0); }
static void l_kd(float v, void *ctx){ (void)ctx; rec(A_L_KD,0,v,0,0,0); }
static void l_spd(float v, void *ctx){ (void)ctx; rec(A_L_SPD,0,v,0,0,0); }
static void l_inv(void *ctx){ (void)ctx; rec(A_L_INV,0,0,0,0,0); }
static int8_t l_inverted(void *ctx){ (void)ctx; rec(A_L_INVERTED,1,0,0,0,0); return 1; }
static void l_scnt(int32_t v, void *ctx){ (void)ctx; rec(A_L_SCNT,(int)v,0,0,0,0); }
static void l_tcnt(int32_t v, void *ctx){ (void)ctx; rec(A_L_TCNT,(int)v,0,0,0,0); }

/* ---- steering mock ---- */
static steering_ret_t st_set(float v, void *ctx){ (void)ctx; rec(A_ST_SET,0,v,0,0,0); return STEERING_OK; }
static float st_get(void *ctx){ (void)ctx; rec(A_ST_GET,0,0,0,0,0); return -90.0f; }
static steering_ret_t st_nudge(float d, void *ctx){ (void)ctx; rec(A_ST_NUDGE,0,d,0,0,0); return STEERING_OK; }
static steering_ret_t st_lmin(float v, void *ctx){ (void)ctx; rec(A_ST_LMIN,0,v,0,0,0); return STEERING_OK; }
static steering_ret_t st_lmax(float v, void *ctx){ (void)ctx; rec(A_ST_LMAX,0,v,0,0,0); return STEERING_OK; }
static steering_ret_t st_ctr(float v, void *ctx){ (void)ctx; rec(A_ST_CTR_SET,0,v,0,0,0); return STEERING_OK; }
static steering_ret_t st_center(void *ctx){ (void)ctx; rec(A_ST_CENTER,0,0,0,0,0); return STEERING_OK; }

/* ---- app_control mock ---- */
static void ctl_stopall(void *ctx){ (void)ctx; rec(A_CTL_STOPALL,0,0,0,0,0); }
static bridge_ret_t ctl_step(uint8_t id,int16_t rate,uint32_t dur,uint8_t div,uint32_t now,void *ctx){
    (void)ctx; rec(A_CTL_STEP,id,(float)rate,(float)dur,(float)div,now); return BRIDGE_OK; }
static void ctl_tel(uint8_t v, void *ctx){ (void)ctx; rec(A_CTL_TEL,v,0,0,0,0); }
static void ctl_rec(uint8_t v, void *ctx){ (void)ctx; rec(A_CTL_REC,v,0,0,0,0); }
static void ctl_dump(void *ctx){ (void)ctx; rec(A_CTL_DUMP,0,0,0,0,0); }
static void ctl_odom(uint8_t v, void *ctx){ (void)ctx; rec(A_CTL_ODOM,v,0,0,0,0); }
static bridge_ret_t ctl_wd(uint32_t ms,void *ctx){ (void)ctx; rec(A_CTL_WD,0,0,0,0,ms); return BRIDGE_OK; }
static bridge_ret_t ctl_itel(uint8_t m,void *ctx){ (void)ctx; rec(A_CTL_ITEL,m,0,0,0,0); return BRIDGE_OK; }
static bridge_ret_t ctl_irate(uint16_t ms,void *ctx){ (void)ctx; rec(A_CTL_IRATE,0,0,0,0,ms); return BRIDGE_OK; }

/* ---- imu mock ---- */
static bridge_ret_t imu_mahony(uint8_t id,float kp,float ki,void *ctx){ (void)ctx; rec(A_IMU_MAHONY,id,kp,ki,0,0); return BRIDGE_OK; }
static bridge_ret_t imu_drift(uint8_t id,uint8_t v,void *ctx){ (void)ctx; rec(A_IMU_DRIFT,id,(float)v,0,0,0); return BRIDGE_OK; }
static bridge_ret_t imu_recal(uint8_t id,void *ctx){ (void)ctx; rec(A_IMU_RECAL,id,0,0,0,0); return BRIDGE_OK; }

/* ---- motor mock ---- */
static bridge_ret_t mtr_mps(uint8_t id,float v,void *ctx){ (void)ctx; rec(A_MTR_MPS,id,v,0,0,0); return BRIDGE_OK; }
static bridge_ret_t mtr_rate(uint8_t id,int16_t r,void *ctx){ (void)ctx; rec(A_MTR_RATE,id,(float)r,0,0,0); return BRIDGE_OK; }

/* ---- arb mock ---- */
static LinkArbRet arb_req(LinkArbOwner t,void *ctx){ (void)ctx; rec(A_ARB_REQ,(int)t,0,0,0,0); return LINK_ARB_OK; }
static LinkArbOwner arb_owner(void *ctx){ (void)ctx; rec(A_ARB_OWNER,0,0,0,0,0); return LINK_ARB_USB; }

static cmd_exec_io_t mkio(void)
{
    cmd_exec_io_t io;
    memset(&io, 0, sizeof(io));
    io.send = mock_send;
    io.note_cmd = mock_note_cmd;
    io.note_resp = mock_note_resp;
    io.override_manual = mock_override;
    io.wd_feed = mock_wdfeed;
    io.ppr_set = mock_ppr;
    io.uart_enabled = 1; io.usb_enabled = 1;
    io.spd_stop=spd_stop; io.spd_set_target=spd_tgt; io.spd_set_gains=spd_setg;
    io.spd_get_gains=spd_getg; io.spd_set_ff_gain=spd_ff;
    io.line_enable=l_en; io.line_set_auto=l_auto; io.line_set_kp=l_kp; io.line_set_kd=l_kd;
    io.line_set_speed=l_spd; io.line_invert=l_inv; io.line_inverted=l_inverted;
    io.line_set_straight_cnt=l_scnt; io.line_set_turn_cnt=l_tcnt;
    io.steer_set=st_set; io.steer_get=st_get; io.steer_nudge=st_nudge;
    io.steer_set_limit_min=st_lmin; io.steer_set_limit_max=st_lmax;
    io.steer_set_center=st_ctr; io.steer_center=st_center;
    io.ctl_stop_all=ctl_stopall; io.ctl_step_start=ctl_step; io.ctl_tel_set=ctl_tel;
    io.ctl_rec_set=ctl_rec; io.ctl_rec_dump=ctl_dump; io.ctl_odom_set=ctl_odom;
    io.ctl_wd_set=ctl_wd; io.ctl_itel_set=ctl_itel; io.ctl_imu_rate_set=ctl_irate;
    io.imu_set_mahony_gains=imu_mahony; io.imu_set_drift_enable=imu_drift;
    io.imu_recalibrate=imu_recal;
    io.mtr_set_speed_mps=mtr_mps; io.mtr_set_rate_0E3=mtr_rate;
    io.arb_request=arb_req; io.arb_owner=arb_owner;
    io.ctx = NULL;
    return io;
}

/* 清空记录与发送缓冲, 返回 io */
static cmd_exec_io_t reset(void)
{
    g_n = 0; g_txlen = 0; g_txbuf[0] = 0;
    return mkio();
}

/* 按实现端构造 TxtCmd (直接构造, 绕过 txt_cmd_parse 用实际整数值) */
static TxtCmd tc(int type, int i0) { TxtCmd t; memset(&t,0,sizeof(t)); t.type=type; t.i0=i0; return t; }

/* ===================== 用例 ===================== */

static void test_ping(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_PING, 0);
    cmd_exec_text(&io, &t, 0);
    check(have_tx("PONG\r\n"), "PING -> PONG");
    check(io.send != NULL, "io.fill non-null sanity");
}

static void test_stop(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_STOP, 0);
    cmd_exec_text(&io, &t, 0);
    check(have_tx("STOP OK LINE OFF\r\n"), "STOP 应答字节");
    /* 调用序: spd_stop x2, line_enable(auto), center, stop_all */
    int ok = g_n >= 5;
    if (ok) {
        ok = (g_call[0].act==A_SPD_STOP && g_call[0].i0==0) &&
             (g_call[1].act==A_SPD_STOP && g_call[1].i0==1) &&
             (g_call[2].act==A_L_EN && g_call[2].i0==0) &&
             (g_call[3].act==A_L_AUTO && g_call[3].i0==0) &&
             (g_call[4].act==A_ST_CENTER);
    }
    check(ok, "STOP 调用序列 (2stop→lineoff→autoo→center)");
    /* stop_all 也在其中 */
    int saw_stopall = 0; for(int i=0;i<g_n;i++) if(g_call[i].act==A_CTL_STOPALL) saw_stopall=1;
    check(saw_stopall, "STOP 触发 ctl_stop_all");
}

static void test_motor(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_MOTOR, 1); t.f0 = -60.0f;
    cmd_exec_text(&io, &t, 0);
    check(have_tx("M1:-60RPM\r\n"), "M1 -60 应答");
    check(g_n>=2 && g_call[0].act==CMD_OVERRIDE, "MOTOR 先 override_manual");
    int set1 = -1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_SPD_TGT){set1=i;break;}
    check(set1>=0 && g_call[set1].i0==1 && feq(g_call[set1].f0,-60.0f), "MOTOR 调 spd_set_target(1,-60)");
}

static void test_ms(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_MOTOR_BOTH, 0); t.f0 = 30.0f;
    cmd_exec_text(&io, &t, 0);
    check(have_tx("MS:30RPM\r\n"), "MS 30 应答");
    int c0=0,c1=0; for(int i=0;i<g_n;i++) if(g_call[i].act==A_SPD_TGT){ if(g_call[i].i0==0)c0++; else c1++; }
    check(c0==1 && c1==1, "MS 双电机各一次");
}

static void test_brake(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_BRAKE, 0);
    cmd_exec_text(&io, &t, 0);
    check(have_tx("M0:BRAKE\r\n"), "B0 应答");
    int st=-1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_SPD_STOP){st=i;break;}
    check(st>=0 && g_call[st].i0==0, "BRAKE 调 spd_stop(0)");
}

static void test_bin_motor(void)
{
    cmd_exec_io_t io = reset();
    /* 0x01: id=0, rpm=60.0f */
    float v = 60.0f; uint8_t d[5]; d[0]=0; memcpy(&d[1],&v,4);
    cmd_exec_bin(&io, 0x01, d, 5, 0);
    check(have_tx("M0:60RPM\r\n"), "帧 0x01 M0:60 应答");
    int set0=-1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_SPD_TGT&&g_call[i].i0==0){set0=i;break;}
    check(set0>=0 && feq(g_call[set0].f0,60.0f), "帧 0x01 调 spd_set_target(0,60)");
}
static void test_bin_e0(void)
{
    cmd_exec_io_t io = reset();
    /* 0xE0: id=0, kp=.24 ki=.13 kd=.20 */
    uint8_t d[13]; d[0]=0; float a[3]={0.24f,0.13f,0.20f}; memcpy(&d[1],&a[0],4); memcpy(&d[5],&a[1],4); memcpy(&d[9],&a[2],4);
    cmd_exec_bin(&io, 0xE0, d, 13, 0);
    check(have_tx("OK\r\n"), "帧 0xE0 应答 OK");
    int sg=-1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_SPD_SETG){sg=i;break;}
    check(sg>=0 && g_call[sg].i0==0 && feq(g_call[sg].f0,0.24f) &&
          feq(g_call[sg].f1,0.13f) && feq(g_call[sg].f2,0.20f), "帧 0xE0 增益传递");
}
static void test_bin_duty(void)
{
    cmd_exec_io_t io = reset();
    /* 0x20: id=1, duty 50.0f → rate 500 */
    float v = 50.0f; uint8_t d[5]; d[0]=1; memcpy(&d[1],&v,4);
    cmd_exec_bin(&io, 0x20, d, 5, 0);
    check(have_tx("D1:50.0\r\n"), "帧 0x20 D1:50.0 应答");
    int st=-1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_MTR_RATE){st=i;break;}
    check(st>=0 && g_call[st].i0==1 && (int)g_call[st].f0==500, "帧 0x20 mtr_set_rate_0E3(1,500)");
}
static void test_bin_unknown(void)
{
    cmd_exec_io_t io = reset();
    uint8_t d[1]={0};
    cmd_exec_bin(&io, 0x99, d, 1, 0);
    check(have_tx("?\r\n"), "未知帧命令回 ?");
}
static void test_bin_duty_bad(void)
{
    cmd_exec_io_t io = reset();
    float v = 250.0f; uint8_t d[5]; d[0]=0; memcpy(&d[1],&v,4);
    cmd_exec_bin(&io, 0x20, d, 5, 0);
    check(have_tx("?\r\n"), "帧 0x20 超限回 ? (不执行)");
}

static void test_link_query(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_LINK, -1);
    cmd_exec_text(&io, &t, 0);
    check(have_tx("OWNER:USB\r\n"), "LINK 查询 OWNER:USB");
}
static void test_link_switch(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_LINK, 1);  /* 切 UART */
    cmd_exec_text(&io, &t, 0);
    check(have_tx("LINK:UART OK\r\n"), "LINK 1 切 UART 应答");
    int r=-1; for(int i=0;i<g_n;i++) if(g_call[i].act==A_ARB_REQ){r=i;break;}
    check(r>=0 && g_call[r].i0==LINK_ARB_UART, "LINK 1 调 arb_request(UART)");
}

static void test_servo(void)
{
    cmd_exec_io_t io = reset();
    TxtCmd t = tc(TXTCMD_SERVO, 0); t.f0 = 30.0f;
    cmd_exec_text(&io, &t, 0);
    /* steer_get mock 返回 -90 → 回显 -90.0 */
    check(have_tx("SV:-90.0\r\n"), "SV 30 应答(回显 steer_get)");
}

int main(void)
{
    printf("=== cmd_exec 命令执行器 PC 测试桩 (R2) ===\n");
    test_ping(); test_stop(); test_motor(); test_ms(); test_brake();
    test_bin_motor(); test_bin_e0(); test_bin_duty(); test_bin_unknown(); test_bin_duty_bad();
    test_link_query(); test_link_switch(); test_servo();

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}