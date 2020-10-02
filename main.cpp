#include <stdlib.h>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <sys/time.h>//time.hとは別物らしい．http://8ttyan.hatenablog.com/entry/2015/02/03/003428
#include <signal.h>
#include <pigpiod_if2.h>
#include <math.h>
#include<unistd.h>
#include "OmniOperator.hpp"
#include "ArmOperator.hpp"
#include <poll.h>
#include "xwiimote.h"
#include <stdlib.h>
#include <errno.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <string.h>
#include <fcntl.h>

#define MSEC 1000000
#define INTERVAL_200MSEC 200*MSEC
#define INTERVAL_50MSEC 50*MSEC
#define INTERVAL_20MSEC 20*MSEC
#define INTERVAL_10MSEC 10*MSEC
#define INTERVAL_1MSEC 1*MSEC
#define PI 3.14159265359
#define MAXPULSE 100

#define abs_(x) ((x)<0.0 ? (-1*(x)) : (x) )

#define PATH_LENGTH 100

using namespace std;

const int SIGNAL_1MS = 1;
const int SIGNAL_10MS = 10;
const int SIGNAL_20MS = 20;
const int SIGNAL_50MS = 50;
const int SIGNAL_200MS = 200;


enum MODE {
    NUNCHUK,
    BALANCEBOARD,
};
MODE actMode = NUNCHUK;
volatile bool quit_flag=false;
const float MIN_WEIGHT = 5; //バランスボードに人が乗っていないと判定する閾値
/*オムニ操作初期化*/
OmniOperator *omniOperator1 = NULL;
const uint8_t TOP_PIN_ID = 4;
const uint8_t LEFT_PIN_ID = 27;
const uint8_t RIGHT_PIN_ID = 22;

/*インジケータ定義*/
const uint8_t LED1_PIN_ID = 6;
const uint8_t LED2_PIN_ID = 5;

/*アーム部分定義*/
ArmOperator *armOperator1 = NULL;
const uint8_t SW_PIN_ID = 24;//B接点(常時導通)
const uint8_t ARM_PIN_ID = 23;//
const uint8_t SERVO_PAN=20;
const uint8_t SERVO_TILT=21;

/*Wiiリモコン関係*/
struct WiiRemoteMain{
	uint8_t _A;
	uint8_t _B;
	uint8_t _UP;
	uint8_t _DOWN;
	uint8_t _LEFT;
	uint8_t _RIGHT;
	uint8_t _MINUS;
	uint8_t _PLUS;
	uint8_t _HOME;
	uint8_t _ONE;
	uint8_t _TWO;

	int32_t _ACC_X;
	int32_t _ACC_Y;
	int32_t _ACC_Z;

	//nunchuk
	int32_t _NC_ACC_X;
	int32_t _NC_ACC_Y;
	int32_t _NC_ACC_Z;
	int32_t _NC_STICK_X;
	int32_t _NC_STICK_Y;
	uint8_t _NC_C;
	uint8_t _NC_Z;

	//motionplus
	int32_t _MP_ACC_X;
	int32_t _MP_ACC_Y;
	int32_t _MP_ACC_Z;
}wmMain;

struct WiiRemoteBB{
	//motionplus
	uint16_t _BB_RIGHT_TOP;
	uint16_t _BB_RIGHT_BOTTOM;
	uint16_t _BB_LEFT_TOP;
	uint16_t _BB_LEFT_BOTTOM;
	uint16_t _BB_TOTAL;
}wmBB;

//過去N個のfloatの平均をとりたいだけのクラス．個数<nの場合は考えないこととする．
class LightFloatQueue{
    static const int MAX=10;
    float value[MAX];
    int cnt;
public:
    LightFloatQueue(){
    };
    void add(float val){
	value[cnt]=val;
	cnt = cnt<(MAX-1) ? cnt+1:0;
    }
    float getAverage(){
	float ret=0.;
	for(int i=0;i<MAX;i++){
	    ret+=value[i];
	}
	return ret/MAX;
    }
};
LightFloatQueue balanceXQueue,balanceYQueue;
float balanceXcriteria,balanceYcriteria;//重心の基準値．init時に10回分のアベレージを入れておく．
float bb_position[2];
/*
 * バランスボードの重心(比率)をx,yそれぞれ返す．
 * 計算式としては，x>0において，y=-((1-x)/(1+x) - 1)
 * x<0においてはその逆．
 * x=0 の時に0で，x limit to 1で 1に漸近する．
 */
void updatePositionFromBalanceBoard(float position[2]) {
    position[0] = ((float)wmBB._BB_RIGHT_TOP + wmBB._BB_RIGHT_BOTTOM) / (wmBB._BB_LEFT_TOP + wmBB._BB_LEFT_BOTTOM);
    if (position[0] > 1)
        position[0] = (((float)wmBB._BB_LEFT_TOP + wmBB._BB_LEFT_BOTTOM) / (wmBB._BB_RIGHT_TOP + wmBB._BB_RIGHT_BOTTOM) * (-1.0)) + 1.0;
    else
        position[0] -= 1;

    position[1] = ((float)wmBB._BB_LEFT_TOP + wmBB._BB_RIGHT_TOP) / (wmBB._BB_LEFT_BOTTOM + wmBB._BB_RIGHT_BOTTOM);
    if (position[1] > 1)
        position[1] = (((float)wmBB._BB_LEFT_BOTTOM + wmBB._BB_RIGHT_BOTTOM) / (wmBB._BB_LEFT_TOP + wmBB._BB_RIGHT_TOP) * (-1.0)) + 1.0;
    else
        position[1] -= 1;

    //履歴の保存
    balanceXQueue.add(position[0]);
    balanceYQueue.add(position[1]);
}

/*
 * バランスボードの重心(重心の傾きなどを補正済)を返す．基本的にはこれだけ使う
 */
void getRevisedPositionFromBalanceBoard(float position[2]) {
    updatePositionFromBalanceBoard(position);
    position[0]-=balanceXcriteria;
    position[1]-=balanceYcriteria;
}

void initCriteriaOfBalanceBoard() {
    balanceXcriteria = balanceXQueue.getAverage();
    balanceYcriteria = balanceYQueue.getAverage();
}

/*
 * 本体加速度によるピッチ角
 */
/*
 * +-1800で返す
 */
int32_t radToDeg(float rad){
	return (int32_t)(rad * 1800/PI);
}
//y軸使うのがよさそう．水平で大体-30,下90°で50,上90°で-130.つまり下にいくと+80，上に行くと100マイナスって感じらしい．
int32_t calc_pitch(){
	int32_t pitch;
	int32_t y=wmMain._ACC_Y;

	y+=30;//水平で30くらいずれているため
	if(y<0){//上向き
		pitch=900-radToDeg(acos(-y/(float)110));//マージン取って+10の110で除算
	}else{//下向き
		pitch=-900+radToDeg(acos(y/(float)90));
	}
	return pitch;
}

/*
 * モーションプラスによるヨー角計算
 */
#define MAX_YAW 4500 //6000で90°くらいなので，45°くらいの動作範囲を想定しつつ，少しマージンとって指定
int32_t totalYaw=0;
void init_yaw_angle(){
	totalYaw=0;
}
/*-1800~1800で返す*/
int32_t calc_yaw(){
	//1000大体6000くらいで90度．(根拠なし)
	return totalYaw*3/20;//90/6000=>3/20 
}
bool caliblate_mp_flag=true;//水平状態の時に初期化ボタンを押された際にtrueになる
//int xwiimote_stopped_flag_cnt=0;//xwiimoteからの入力が止まってしまったことを検出した際に+1
//bool xwiimote_stopped_flag=false;//xwiimoteからの入力が止まってしまったことを検出した際にtrueになる

void interruptedFunc(int sig, siginfo_t *si, void *uc);

void timer_init(int interval, int sigId) {
    //シグナルに番号を付与できるモダンな？やりかたに変更
    //http://umezawa.dyndns.info/wordpress/?p=3174
    struct itimerspec timerSpec;
    struct sigevent se;
    timerSpec.it_value.tv_nsec = interval;
    timerSpec.it_value.tv_sec = 0;
    timerSpec.it_interval.tv_nsec = interval;
    timerSpec.it_interval.tv_sec = 0;

    timer_t timerid_signal;
    memset(&se, 0, sizeof (se));
    se.sigev_value.sival_int = sigId;
    se.sigev_signo = SIGALRM;
    se.sigev_notify = SIGEV_SIGNAL;
    timer_create(CLOCK_REALTIME, &se, &timerid_signal);
    timer_settime(timerid_signal, 0, &timerSpec, NULL);
}

void interupt_init() {
    struct sigaction action;
    memset(&action, 0, sizeof (action)); /* actionの内容を0でクリア */

    action.sa_sigaction = interruptedFunc; //呼び出す関数のセット．こっちの方式だと呼び出し元で設定した番号を引ける
    action.sa_flags = SA_SIGINFO; //sa_handlerではなくsa_sigactinを指定するための設定．
    action.sa_flags |= SA_RESTART; /* システムコールが中止しない */
    sigemptyset(&action.sa_mask); //maskの中身を0にクリア．maskに登録しておくと割り込みが重なった時に保留してくれる．
    if (sigaction(SIGALRM, &action, NULL) < 0)//第一引数でシグナルの種類を指定．今回はアラームの時間満了で呼ばれる．第三引数にsigaction型をセットしておくと，ひとつ古い値が返ってくる．今回は破棄
    {
        perror("sigaction error");
        exit(1);
    }
}

float addReviseRotation(float rate = 0.3) {
    if (actMode == NUNCHUK) {
        return 0;
    }
	
	//以下，actModeがBalanceBoardのとき

	float x_val=float(wmMain._NC_STICK_X)/100;//-1~1の値が得られる
    if (abs_(x_val) < 5.0 / 100) {
        x_val = 0;
    }
    return -x_val*rate;
}

int calcByNunchuk( float xyVals[2]) {

    float x_val = float(wmMain._NC_STICK_X) / 100; //-1~1の値が得られる
    float y_val = float(wmMain._NC_STICK_Y) / 100;

    //printf("%f\t",x_val);
    /*ニュートラルのずれを無視*/
    if (abs_(x_val) < 5.0 / 100) {
        x_val = 0;
    }
    if (abs_(y_val) < 5.0 / 100) {
        y_val = 0;
    }
    xyVals[0] = x_val;
    xyVals[1] = y_val;
    return 0;
}

//uint32_t prev_mp_x=0,prev_mp_y=0,prev_mp_z=0;
void interruptedFunc(int sig, siginfo_t *si, void *uc) {
    static uint8_t buttonA = 0;
    static uint8_t buttonC = 0;
    switch (si->si_value.sival_int) {
        case SIGNAL_10MS:
            break;
		case SIGNAL_20MS:
			break;
//		case SIGNAL_200MS:
		case SIGNAL_50MS:
			//バランスボード重量
//			printf("bbW:%05hu\tbbX:%05hu\tbbY:%05hu\tbbZ:%05hu\r\n",wmBB._BB_LEFT_TOP,wmBB._BB_RIGHT_TOP,wmBB._BB_LEFT_BOTTOM,wmBB._BB_RIGHT_BOTTOM);
			//ヌンチャク
//			printf("ncX:%03d\tncY:%03d\r\n",wmMain._NC_STICK_X,wmMain._NC_STICK_Y);
			//モーションプラス+ヌンチャク
//			printf("ncX:%03d\tncY:%03d\tmpX:%05d\tmpY:%05d\tmpZ:%05d\r\n",wmMain._NC_STICK_X,wmMain._NC_STICK_Y,wmMain._MP_ACC_X,wmMain._MP_ACC_Y,wmMain._MP_ACC_Z);
			//モーションプラス
//			printf("mpX:%05d\tmpY:%05d\tmpZ:%05d\r\n",wmMain._MP_ACC_X,wmMain._MP_ACC_Y,wmMain._MP_ACC_Z);
			//ピッチ角，ヨー角
			printf("pitch=%04d,yaw:%04d\r\n",calc_pitch(),calc_yaw());
			armOperator1->setTilt(calc_pitch());
			armOperator1->setPan(calc_yaw());

			//Wiiリモコンからの入力がストップしてしまったケースの検出．mp3つとも値が変わっていなければまず確実．初期化中などは値が0のまま取得されてしまうので無視
			//と思ったが稀に誤動作するので，2連続で条件を満たした際に検出することにする
//			if(wmMain._MP_ACC_X!=0 &&wmMain._MP_ACC_Y!=0 &&wmMain._MP_ACC_Z!=0
//					&& prev_mp_x==wmMain._MP_ACC_X && prev_mp_y==wmMain._MP_ACC_Y && prev_mp_z==wmMain._MP_ACC_Z ) {
//				if(++xwiimote_stopped_flag_cnt>=2){
//					xwiimote_stopped_flag=true;
//					puts("fputs called");
	//				fputs("z\r\n",stdin);
//				}
//			}else{
//				xwiimote_stopped_flag_cnt=0;
//			}
//			prev_mp_x=wmMain._MP_ACC_X; prev_mp_y=wmMain._MP_ACC_Y; prev_mp_z=wmMain._MP_ACC_Z;

			armOperator1->checkArmState();//アームの伸縮停止判定件WDタイマ

            float xyVals[2] = {0., 0.};
            switch (actMode) {
                case NUNCHUK:
                    calcByNunchuk( xyVals);
                    break;
                case BALANCEBOARD:
					//センターポイント初期化処理．こっちで調整するより，立ち位置見直してもらったほうが事故が少ない．
					/*
                    //ボタン押下処理．暫定対策．
                    if ((buttonA^(wiimote1.state.buttons & CWIID_BTN_A)) && (wiimote1.state.buttons & CWIID_BTN_A)) {
                        wiimote2.initCriteriaOfBalanceBoard();
                        puts("INIT CALLED");
                    } else if ((buttonC^(wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C)) && (wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C)) {
                        wiimote2.initCriteriaOfBalanceBoard();
                        puts("INIT CALLED");
                    }
                    buttonA = wiimote1.state.buttons & CWIID_BTN_A;
                    buttonC = wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C;
					 */

					getRevisedPositionFromBalanceBoard(xyVals);
                    if (!wmMain._NC_Z || wmBB._BB_TOTAL<MIN_WEIGHT) {//ボタンを押していない，または重量が閾値を超えていない場合は停止
                        xyVals[0] = xyVals[1] = 0;
                    }
                    break;
            }
            float rotation = addReviseRotation();

            /*モーター回転量計算*/
            omniOperator1->move(xyVals[0], xyVals[1], rotation);
			break;
    }
}

/*エンターを押した際にたまるらしい*/
int isReceived(int fd){
	fd_set fdset;
	struct timeval timeout;
	FD_ZERO(&fdset);
	FD_SET(fd,&fdset);
	timeout.tv_sec=0;
	timeout.tv_usec=0;
	return(select(fd+1,&fdset,NULL,NULL,&timeout));
}
int receive(int fd, char *buff, int buffSize){
	int i= isReceived(fd);
	if(i){
		read(fd,buff,buffSize);
		buff[strlen(buff)-1]='\0';
	}
	return (i);
}

class IndicatorOperator{
private:
	const uint8_t PI_ID;
	const uint8_t LED_PIN;
public:
	IndicatorOperator(uint8_t piId, uint8_t ledPinId):PI_ID(piId),LED_PIN(ledPinId){
		init();
	}
	void init(){
		set_mode(PI_ID, LED_PIN, PI_OUTPUT);
		set_pull_up_down(PI_ID, LED_PIN, PI_PUD_OFF);
		set_PWM_frequency(PI_ID, LED_PIN, 1); //1Hz//pigpioの仕様上，下限は5Hzらしい．
		set_PWM_range(PI_ID, LED_PIN, 255); //点灯/点滅/消灯だけできればよいので適当.255はデフォルト値なので書かなくてもいいはず
		gpio_write(PI_ID,LED_PIN,0);
	}
	void startFastBrink(){
		set_PWM_frequency(PI_ID, LED_PIN, 1); //1Hz//pigpioの仕様上，下限は5Hzらしい．
		set_PWM_dutycycle(PI_ID, LED_PIN, 127); //点滅開始
	}
	void startSlowBrink(){
		set_PWM_frequency(PI_ID, LED_PIN, 5); //5Hz
		set_PWM_dutycycle(PI_ID, LED_PIN, 127); //点滅開始
	}
	
	void putOff(){
		gpio_write(PI_ID,LED_PIN,0);
	}

	void putOn(){
		set_PWM_dutycycle(PI_ID, LED_PIN, 255); //点滅開始
	}
};

//エラー発生時にコール．エラー点滅ののちプログラム終了
void quitProgram(int piId, IndicatorOperator *greenLedOperator,IndicatorOperator *redLedOperator,int ret) {
	greenLedOperator->startSlowBrink();
	redLedOperator->startSlowBrink();

    time_sleep(3);
	greenLedOperator->putOff();
	redLedOperator->putOff();
//	delete(greenLedOperator);
//	delete(redLedOperator);
    pigpio_stop(piId);
    exit(ret);
}
static char *get_dev(int num)
{
	struct xwii_monitor *mon;
	char *ent;
	int i = 0;

	mon = xwii_monitor_new(false, false);
	if (!mon) {
		printf("Cannot create monitor\n");
		return NULL;
	}

	while ((ent = xwii_monitor_poll(mon))) {
		if (++i == num)
			break;
		free(ent);
	}

	xwii_monitor_unref(mon);

	if (!ent)
		printf("Cannot find device with number #%d\n", num);

	return ent;
}
static struct xwii_iface *iface[2];

/* device watch events */

//static void handle_watch(void)
//{
//	static unsigned int num;
//	int ret;

//	printf("Info: Watch Event #%u", ++num);

//	ret = xwii_iface_open(iface, xwii_iface_available(iface) |
//				     XWII_IFACE_WRITABLE);
//	if (ret)
//		printf("Error: Cannot open interface: %d", ret);

//}

static void set_keys(const struct xwii_event *event)
{
	unsigned int code = event->v.key.code;
	bool pressed = event->v.key.state;
	char *str = NULL;

	switch(code){
		case XWII_KEY_LEFT:
			break;
		case XWII_KEY_RIGHT:
			break;
		case XWII_KEY_DOWN:
			break;
		case XWII_KEY_UP:
			break;
		case XWII_KEY_A:
			if(pressed){
				wmMain._A=1;
				init_yaw_angle();//これがsetPanMiddle()に相当
				armOperator1->setTiltMiddle(calc_pitch());
				printf("pitch middle = %d\r\n",armOperator1->getTiltMiddle());
			}else{
				wmMain._A=0;
			}
			break;
		case XWII_KEY_B:
			if(pressed){
				armOperator1->extendArm();
				puts("arm extended");
			}else{
				armOperator1->shortenArm();
				puts("arm shortened");
			}
			break;
		case XWII_KEY_PLUS:
			break;
		case XWII_KEY_MINUS:
			break;
		case XWII_KEY_HOME:
			if(pressed){
				wmMain._HOME=1;
				caliblate_mp_flag=true;
			}else{
				wmMain._HOME=0;
			}
			break;
		case XWII_KEY_ONE:
			break;
		case XWII_KEY_TWO:
			break;
	}
}

static void set_accel(const struct xwii_event *event){
	wmMain._ACC_X=event->v.abs[0].x;
	wmMain._ACC_Y=event->v.abs[0].y;
	wmMain._ACC_Z=event->v.abs[0].z;
}

static void set_nunchuk(const struct xwii_event *event){
	if (event->type == XWII_EVENT_NUNCHUK_MOVE) {
		wmMain._NC_ACC_X=event->v.abs[1].x;
		wmMain._NC_ACC_Y=event->v.abs[1].y;
		wmMain._NC_ACC_Z=event->v.abs[1].z;

		wmMain._NC_STICK_X=event->v.abs[0].x;
		wmMain._NC_STICK_Y=event->v.abs[0].y;
	}
	if (event->type == XWII_EVENT_NUNCHUK_KEY) {
		if (event->v.key.code == XWII_KEY_C) {
			wmMain._NC_C=event->v.key.state;
		} else if (event->v.key.code == XWII_KEY_Z) {
			wmMain._NC_Z=event->v.key.state;
		}
	}
}
static void set_motionplus(const struct xwii_event *event){
	wmMain._MP_ACC_X=event->v.abs[0].x;
	wmMain._MP_ACC_Y=event->v.abs[0].y;
	wmMain._MP_ACC_Z=event->v.abs[0].z;

	totalYaw+=wmMain._MP_ACC_X/100;
	if(totalYaw>MAX_YAW){
		totalYaw=MAX_YAW;
	}else if(totalYaw<-MAX_YAW){
		totalYaw = -MAX_YAW;
	}
}

static void calibrate_motionplus(struct xwii_iface *iface){
	int32_t x, y, z, factor;

	xwii_iface_get_mp_normalization(iface, &x, &y, &z, &factor);
	x = wmMain._MP_ACC_X + x;
	y = wmMain._MP_ACC_Y + y;
	z = wmMain._MP_ACC_Z + z;
	factor=50;//xwiishowでのデフォルト係数
	xwii_iface_set_mp_normalization(iface, x, y, z, factor);
}


/*
 * xwiidで取得できる値はすでに補正済みっぽい
 */
static void set_balanceboard(const struct xwii_event *event){
	wmBB._BB_RIGHT_TOP=event->v.abs[0].x;
	wmBB._BB_RIGHT_BOTTOM=event->v.abs[1].x;
	wmBB._BB_LEFT_TOP=event->v.abs[2].x;
	wmBB._BB_LEFT_BOTTOM=event->v.abs[3].x;

	wmBB._BB_TOTAL=wmBB._BB_LEFT_BOTTOM+wmBB._BB_LEFT_TOP+wmBB._BB_RIGHT_BOTTOM+wmBB._BB_RIGHT_TOP;
	updatePositionFromBalanceBoard(bb_position);
}

//static int run_iface(struct xwii_iface *iface)
static int run_iface(struct xwii_iface *iface[],const int device_num)
{
	struct xwii_event event;
	int ret = 0;
	int fds_num=device_num+1;//標準入力分をプラス
	struct pollfd fds[3];//device_numが1の時は，3つめは使われない

	memset(fds, 0, sizeof(fds));
	fds[0].fd = 0;
	fds[0].events = POLLIN;
	for(int i=0;i<device_num;i++){
		fds[i+1].fd = xwii_iface_get_fd(iface[i]);
		fds[i+1].events = POLLIN;

	}

	for(int i=0;i<device_num;i++){
		ret = xwii_iface_watch(iface[i], true);
		if (ret)
			puts("Error: Cannot initialize hotplug watch descriptor");
	}
//	curs_set(0);
//	timeout(0);//getch()のタイムアウトを0にする．

	sigset_t sigmask;
	ret=sigaddset(&sigmask,SIGALRM);
	if(ret<0){
		puts("sgaddset failed");
	}
	fcntl(0,F_SETFL,O_NONBLOCK);//標準入力をノンブロッキングに
	char buff[10];
	while(true){
//		ret = poll(fds, fds_num, 0);//標準入力含め，変化があるまで待機．
		struct timespec timeout;
		timeout.tv_nsec = 40*1000*1000;//40ms
		timeout.tv_sec = 0;
		ret = ppoll(fds, fds_num, &timeout,&sigmask);//標準入力含め，変化があるまで待機．タイマ割り込みで待機から抜けないようマスク
		if (ret < 0) {
			if (errno != EINTR) {//待機中にシグナル発生したらEINTRが呼ばれる．
				ret = -errno;
				printf("Error: Cannot poll fds: %d", ret);
				return ret;
			}else{
				puts("eintr occurred");//ppollで適切にマスクしている場合，呼ばれないはず．

			}
		}
//		puts("polled");
		if(ret==0){//timeout発生時．※正常時はイベントが発生したfd数=正が返ってきている
			puts("timeout occured");
//			if(xwiimote_stopped_flag){
//				puts("xwiimote stopped!!");
//				return -1;
//			}
		}

		for(int i=0;i<device_num;i++){
			ret = xwii_iface_dispatch(iface[i], &event, sizeof(event));
			if (ret) {
				if (ret != -EAGAIN) {
					printf("Error: Read failed with err:%d", ret);
					return ret;
				}
			} else {
				switch (event.type) {
				case XWII_EVENT_GONE:
					puts("Info: Device gone");
					fds[i+1].fd = -1;
					fds[i+1].events = 0;
					fds_num -=1;
					break;
				case XWII_EVENT_WATCH:
					puts("Info: watch happened");
//					handle_watch();
					break;
				case XWII_EVENT_KEY:
					set_keys(&event);
					break;
				case XWII_EVENT_ACCEL:
					set_accel(&event);
					break;
				case XWII_EVENT_NUNCHUK_KEY:
				case XWII_EVENT_NUNCHUK_MOVE:
					set_nunchuk(&event);
					break;
				case XWII_EVENT_MOTION_PLUS:
					set_motionplus(&event);
					if(caliblate_mp_flag){
						calibrate_motionplus(iface[i]);//i番目がどちらかは不明だが，mpイベントが発生した方なら確実
						caliblate_mp_flag=false;
					}
					break;
				case XWII_EVENT_BALANCE_BOARD:
					set_balanceboard(&event);
					break;
				default:
					puts("unexpected event occured");
					break;
				}
			}
		}


//		puts("dispatched");
		char buff[10];
//		read(fd,buff,sizeof(buff));
//		buff[strlen(buff)-1]='\0';
//		if(receive(0,buff,sizeof(buff))){
		if(read(0,buff,sizeof(buff))){
			switch(buff[0]){
				case 'q':
					puts("finish");
					return(0);
				case 'i'://initialize balance board
					initCriteriaOfBalanceBoard();
					break;
				case 'm':
					caliblate_mp_flag=true;
					break;
				case 'x':
					init_yaw_angle();
					break;
				case 'z':
					puts("received puts");
					break;
			}
		}
//		puts("received");
	}

	return ret;
}
/*
 * 可変にするの面倒なので，2つまで取得
 * 取得できたwiimoteのパスを返す
 */
static int get_all_device_paths(char paths[][100], int *deviceNum)
{
	struct xwii_monitor *mon;
	char *ent;
	int num = 0;

	mon = xwii_monitor_new(false, false);
	if (!mon) {
		printf("Cannot create monitor\n");
		return -EINVAL;
	}

	while ((ent = xwii_monitor_poll(mon))) {
		//*(paths[num]+0) == paths[num][0]
		//ということは，paths[num]+0 == paths[num] だが，これは*(char[100])で，*charとは別物らしい．
		//左辺をアドレス指定すると，それは定数扱いになって，5=10みたいな計算をしようとしていることと同じになるらしく，エラーが出る．
		//なので苦肉の策としてstrcpyを使う
		strcpy(paths[num],ent);
//		paths+num*PATH_LENGTH=ent;
		printf("  Found device #%d: %s\n", ++num, ent);
		free(ent);
		*deviceNum=num;
		if(num==2){
			break;
		}
	}

	xwii_monitor_unref(mon);
	return 0;
}

int main(void) {
    int piId = pigpio_start(NULL, NULL); //ネットワーク越しに使えるっぽい．NULL指定時はローカルホストの8888ポート

    //起動確認LEDのセットアップ
	IndicatorOperator greenLedOperator(piId,LED1_PIN_ID);
	IndicatorOperator redLedOperator(piId,LED2_PIN_ID);

    //オムニ操作部分のセットアップ
    omniOperator1 = new OmniOperator(piId, TOP_PIN_ID, LEFT_PIN_ID, RIGHT_PIN_ID);
    omniOperator1->init(300, 1000); //1kHz,分解能1000
    omniOperator1->set_limit(60); //最大出力を60%に制限

	/*アーム部分のセットアップ*/
	armOperator1 = new ArmOperator(piId,SERVO_TILT,SERVO_PAN,ARM_PIN_ID,SW_PIN_ID);
	armOperator1->init();
	
//	return (0);

	/*xwiimote関係*/
	puts("start xwiimote initialition");
	char paths[2][100];//パス長ってどのくらいか不明だが，1000取っておけば多分大丈夫でしょう．
//	char paths[PATH_LENGTH*2];//どうも多次元配列がうまくいかないので妥協
	int device_num=0;

	/*パスの取得*/
	int ret= get_all_device_paths(paths,&device_num);
	if(ret!=0){
		puts("failed to get any device path");
		quitProgram(piId,&greenLedOperator,&redLedOperator,ret);
		return ret;
	}
	if(device_num==0){
		puts("there are not connected devices");
		quitProgram(piId,&greenLedOperator,&redLedOperator,ret);
		return ret;
	}

	printf("%d devices detected!\r\n",device_num);

	/*インターフェースの作成*/
	for(int i=0;i<device_num;i++){
		ret = xwii_iface_new(&iface[i], paths[i]);
		if (ret) {
			printf("Cannot create xwii_ifaces No.%d\r\n",  i);
			quitProgram(piId,&greenLedOperator,&redLedOperator,ret);
			return ret;
		}
	}

	/*接続確立*/
	for(int i=0;i<device_num;i++){
		ret = xwii_iface_open(iface[i], xwii_iface_available(iface[i]) | XWII_IFACE_WRITABLE);
		if (ret){
			printf("Error: Cannot open interface: %d\r\n", i);
			quitProgram(piId,&greenLedOperator,&redLedOperator,ret);
			return ret;

		}else{
			printf("connected device No.%d\r\n",i);
		}
	}

	interupt_init();
//	timer_init(INTERVAL_200MSEC, SIGNAL_200MS);
	timer_init(INTERVAL_50MSEC, SIGNAL_50MS);
//    timer_init(INTERVAL_20MSEC, SIGNAL_20MS);
	ret = run_iface(iface,device_num);

	/*後処理*/
	for(int i=0;i<device_num;i++){
		xwii_iface_unref(iface[i]);
	}

	greenLedOperator.putOff();
	redLedOperator.putOff();
    pigpio_stop(piId);
    delete(omniOperator1);

	puts("Script finished");

	if(ret!=0){
		quitProgram(piId,&greenLedOperator,&redLedOperator,ret);
	}
	return 0;

}
