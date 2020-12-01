//Varijable za kontroliranje performanca ruke
// vrijeme je u s*e-4
#define PAUZA 95
#define PetiMotor 12
#define CetvrtiMotor 13
#define A 5 // korak akc promjenjivanja vremena čekanja
#define STEPSA 15 // broj koraka akc
#define IMPULS 5
#define POCETNI_FI 0
#define POCETNI_ALFA 27  //158
#define POCETNI_BETA 90  //190
#define POCETNI_ROLL 90
#define GRIPPER_MIN  0
#define GRIPPER_MAX  50
//nepromjenivi mehanički podatci robotske ruke
static const float PIE=3.14159265359;
static const float  C=76.86; //pozicija nul tocke kordinatnog sustava 
static const float  KRAK1=140; // duljina krakova ruke
static const float  KRAK2=130;
static const float  RAD_TO_DEGRESS=57.29577951;
//redukcijski omjeri motora  (kut-->korak)
static const float AFLA_STEP=0.46;
static const float BETA_STEP=0.46;
static const float FI_STEP=0.45; //360/1000

//globalne varijable
int i=0; //counter
float x, y, z;
float fi=POCETNI_FI, finew; 
float alfa=POCETNI_ALFA, alfanew;
float beta=POCETNI_BETA, betanew;
int roll;
float GripperPostotak;
int gripper=GRIPPER_MIN;
float b; //varijabla za racunanje kutova
float l;
bool start=false;
unsigned  long vrijeme=0;

struct stepper {
  bool pin;
  int korak;
  int vrijemekoraka;
  int ukupnikoraci;
  int cekanje [STEPSA*2];
  int stepPin;
  int dirPin;
  int period;
};
