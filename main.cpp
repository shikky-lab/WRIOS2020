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

#define MSEC 1000000
#define INTERVAL_200MSEC 200*MSEC
#define INTERVAL_50MSEC 50*MSEC
#define INTERVAL_20MSEC 20*MSEC
#define INTERVAL_10MSEC 10*MSEC
#define INTERVAL_1MSEC 1*MSEC
#define PI 3.14159265359
#define MAXPULSE 100

#define abs_(x) ((x)<0.0 ? (-1*(x)) : (x) )


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
MODE actMode = BALANCEBOARD;
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

void interruptedFunc(int sig, siginfo_t *si, void *uc) {
    static uint8_t buttonA = 0;
    static uint8_t buttonC = 0;
    switch (si->si_value.sival_int) {
        case SIGNAL_10MS:
            break;
		case SIGNAL_20MS:
			break;
		case SIGNAL_200MS:
			printf("ncX:%03d\tncY:%03d\tmpX:%05d\tmpY:%05d\tmpZ:%05d\r\n",wmMain._NC_STICK_X,wmMain._NC_STICK_Y,wmMain._MP_ACC_X,wmMain._MP_ACC_Y,wmMain._MP_ACC_Z);
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

#define SW_PUSHED (gpio_read(piId,SW_PIN_ID))
#define SW_RELEASED (!gpio_read(piId,SW_PIN_ID))
void moveArm(int piId){
	if(SW_RELEASED){
		set_PWM_dutycycle(piId,ARM_PIN_ID,10);
		while(SW_RELEASED){
		}
	}else{
		set_PWM_dutycycle(piId,ARM_PIN_ID,990);
		usleep(500*1000);
	}
	set_PWM_dutycycle(piId,ARM_PIN_ID,500);
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
void quitProgram(int piId, IndicatorOperator &greenLedOperator,IndicatorOperator &redLedOperator ) {
	greenLedOperator.startSlowBrink();
	redLedOperator.startSlowBrink();

    time_sleep(3);
	greenLedOperator.putOff();
	redLedOperator.putOff();
//	delete(greenLedOperator);
//	delete(redLedOperator);
    pigpio_stop(piId);
    exit(0);
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
static struct xwii_iface *iface;

/* device watch events */

static void handle_watch(void)
{
	static unsigned int num;
	int ret;

	printf("Info: Watch Event #%u", ++num);

	ret = xwii_iface_open(iface, xwii_iface_available(iface) |
				     XWII_IFACE_WRITABLE);
	if (ret)
		printf("Error: Cannot open interface: %d", ret);

}
static void key_show(const struct xwii_event *event)
{
	unsigned int code = event->v.key.code;
	bool pressed = event->v.key.state;
	char *str = NULL;

	if (code == XWII_KEY_LEFT) {
		puts("L");
	} else if (code == XWII_KEY_RIGHT) {
		puts("R");
	} else if (code == XWII_KEY_UP) {
		puts("U");
	} else if (code == XWII_KEY_DOWN) {
		puts("D");
	} else if (code == XWII_KEY_A) {
		if (pressed)
			puts("A pressed");
		else
			puts("A released");
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
}

int init_iface(struct xwii_iface *iface){
	struct xwii_event event;
	int ret = 0, fds_num;
	struct pollfd fds[2];

	//対象のfdが二枠なには，extensionを挿した際にfds[1]に入ってくるから？
	memset(fds, 0, sizeof(fds));
	fds[0].fd = 0;
	fds[0].events = POLLIN;
	fds[1].fd = xwii_iface_get_fd(iface);
	fds[1].events = POLLIN;
	fds_num = 2;

	ret = xwii_iface_watch(iface, true);
	if (ret)
		puts("Error: Cannot initialize hotplug watch descriptor");
}

static int run_iface(struct xwii_iface *iface,struct pollfd fds[2],int fds_num)
{
	ret = poll(fds, fds_num, -1);
	if (ret < 0) {
		if (errno != EINTR) {
			ret = -errno;
			printf("Error: Cannot poll fds: %d", ret);
			break;
		}
	}

	ret = xwii_iface_dispatch(iface, &event, sizeof(event));
	if (ret) {
		if (ret != -EAGAIN) {
			printf("Error: Read failed with err:%d", ret);
			break;
		}
	} else {
		switch (event.type) {
		case XWII_EVENT_GONE:
			puts("Info: Device gone");
			fds[1].fd = -1;
			fds[1].events = 0;
			fds_num = 1;
			break;
		case XWII_EVENT_WATCH:
			handle_watch();
			break;
		case XWII_EVENT_KEY:
			key_show(&event);
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
			break;
		}
	}

	return ret;
}
int main(void) {
    int piId = pigpio_start(NULL, NULL); //ネットワーク越しに使えるっぽい．NULL指定時はローカルホストの8888ポート

    //起動確認LEDのセットアップ
	IndicatorOperator greenLedOperator(piId,LED1_PIN_ID);
	IndicatorOperator redLedOperator(piId,LED2_PIN_ID);

    //オムニ操作部分のセットアップ
    omniOperator1 = new OmniOperator(piId, TOP_PIN_ID, LEFT_PIN_ID, RIGHT_PIN_ID);
    omniOperator1->init(1000, 1000); //1kHz,分解能1000
    omniOperator1->set_limit(60); //最大出力を90%に制限

	/*アーム部分のセットアップ*/
	armOperator1 = new ArmOperator(piId,SERVO_TILT,SERVO_PAN,ARM_PIN_ID,SW_PIN_ID);

	/*xwiimote関係*/
	puts("start xwiimote initialition");
	char *path = NULL;
	path = get_dev(1);
	int ret = xwii_iface_new(&iface, path);
	free(path);
	if (ret) {
		printf("Cannot create xwii_ifaces err:%d\r\n",  ret);
	} else {
		ret = xwii_iface_open(iface, xwii_iface_available(iface) | XWII_IFACE_WRITABLE);
		if (ret)
			printf("Error: Cannot open interface: %d\r\n", ret);
		else{
			puts("connected");
		}

		interupt_init();
		timer_init(INTERVAL_200MSEC, SIGNAL_200MS);
		ret = run_iface(iface);
	}
//    timer_init(INTERVAL_20MSEC, SIGNAL_20MS);

	char buff[10];
	bool quitFlag=false;
	int prevSW=1;
	
	while(!quitFlag){
		float xyVals[2] = {0., 0.};
		if(receive(0,buff,sizeof(buff))){
			switch(buff[0]){
				case 'a': 
					puts("moveArmStart");
					moveArm(piId);
					puts("moveArmFinished");
					break;
				case 'b': 
					puts("B");
					break;
				case 'c':
					puts("start motionplus caliburation");
					break;
				case 'q':
					puts("finishing");
					quitFlag=true;
					break;
			}
		}
		
		int currentSW=gpio_read(piId,SW_PIN_ID);
	}


	/*後処理*/
	xwii_iface_unref(iface);

	greenLedOperator.putOff();
	redLedOperator.putOff();
    pigpio_stop(piId);
    delete(omniOperator1);

	puts("Script finished");

    return 0;
}
