#ifndef CARAXIS_H
#define CARAXIS_H
#include <QCoreApplication>
#include <QDateTime>
#define WIN_SMP_PROCESS 1500000
#define MAX_NUM_AXIS_GR 5

typedef struct CarAxisDetect
{
    int		coord_ax_signal;
    double	pow_ax_signal;
    double	pow_ax_signal_approx;
} CarAxisDetect;

typedef struct CarDetect
{
    int		coord_ax_signal;
    double	pow_ax_signal;
    double	pow_ax_signal_approx;
} CarDetect;


typedef struct CrackDetect
{
    int		coord_crack_signal;
    double	pow_crack_signal;
    double	pow_crack_signal_approx;
} CrackDetect;

typedef struct Pulses
{
    int coord_pulse;
    double pow_pulse;
}Pulses;

typedef struct PulseGroup
{
    int num_pulse_group;
    double freq_stat;
    double freq;
    bool IsSelect = false;
//    std::vector<Pulses>::iterator it_pulses_max_freq;
    int num_pulses_max_freq_stat;
    std::vector<Pulses> pulses;
} PulseGroup;

typedef struct UnionPulseGroup
{
    int num_pulse_group;
    double freq_stat;
    double freq;
    bool IsLattAfterStript;
//    std::vector<Pulses>::iterator it_pulses_max_freq;
    int num_pulses_max_freq_stat;
    std::vector<Pulses> pulses_stript;
    std::vector<Pulses> pulses_latt;
} UnionPulseGroup;

typedef struct CarAxisInfo{
    int num_ax;
    int coord_axis;
    int abs_coord_beg_latt;
    int abs_coord_end_latt;
    double weit_axis;
    double weit_ax_abs;
//    std::vector<int> coord_axis;
//    std::vector<double> weit_axis;
}CarAxisInfo;

typedef struct CarInfo{
    int num_car;
    int num_pulse_group;
    int type;
    double speed;
    QDateTime date_time;
    std::vector<CarAxisInfo> AxisInfo;
}CarInfo;

#include <mutex>
class ProgressBarHelper : public QObject
{
    Q_OBJECT
public:
    ProgressBarHelper(QObject *parent = nullptr) : QObject(parent)
    {
        m_progress = 0;
    }
    ~ProgressBarHelper(){}

    void emitProgress(int progressPortion)
    {
        std::lock_guard<std::mutex> lock(m_progressMutex);
        m_progress += progressPortion;
        if(m_progress > 100) m_progress = 100;
        emit progress(m_progress);
    }

    void resetProgress()
    {
        std::lock_guard<std::mutex> lock(m_progressMutex);
        m_progress = 0;
    }

signals:
    void progress(int p);

public slots:

private:
    int m_progress;
    std::mutex m_progressMutex;
};


void fnFillCarAxisVec(std::vector<CarAxisDetect> & CarAxis, int * coord, int num_coord, double * pow_sign, double * pow_sign_apr);
void fnFillCrackVec(std::vector<CrackDetect> & Crack, int * coord, int num_coord, double * pow_sign, double * pow_sign_apr);

//void fnGetCarPowCenter(int* S, size_t LenS, size_t ProcWnd, size_t LenWndPow, size_t LenWndApprox, size_t FreqWnd, size_t ThLenPulseGroup, bool StrOrLat, std::vector<PulseGroup> & pulse_groups, int mean_powr=0);
void fnGetCarPowCenter(const std::vector<int32_t> &data, size_t ProcWnd, size_t LenWndPow, size_t LenWndApprox, size_t FreqWnd, size_t ThLenPulseGroup, bool StrOrLat, std::vector<PulseGroup> & pulse_groups, std::vector<double> & signal_abs_flt, int mean_powr=0);

void fnCarAxisSelect(std::vector<CarAxisDetect> CarAxis, std::vector<CrackDetect> Crack, size_t LenS, double th0, double th1, int thLen1, int thLen, std::vector<CarAxisDetect> * GroupAx, int & Is_carInterf);
void fnCarAxisSelectSmall(std::vector<CarAxisDetect> CarAxis, std::vector<CrackDetect>  Crack, size_t LenS, double th0, double th1, int thLen1, int thLen, std::vector<CarAxisDetect> * GroupAx, int & Is_carInterf);
void fnSelectAxisGroup(int* S, int LenS, double FreqMax, int LenSpWnd, int LenWndS, int LenWndPow, int LenWndApprox, bool FreqBand, bool IsTruc, int NumCall, std::vector<CarAxisDetect> * GroupAx, int & Shift, double * Q);

//void fnSelectAxisGroupRes(std::vector<CarAxisDetect> * GroupAxFreqBnd_0, std::vector<CarAxisDetect> * GroupAxFreqBnd_1, std::vector<CarAxisDetect> * GroupAx);
void fnSelectAxisGroupRes(std::vector<int32_t> data, double FreqMax, int LenSpWnd, int LenWndS, int LenWndPow, int LenWndApprox, int IsTruc, bool & Is_AxisFound, std::vector<CarAxisDetect> * GroupAx, int & Shift);
void fnAddSelectAxForSmallCar(bool & Is_AxisFound, std::vector<CarAxisDetect> * GroupAx);

void fnUnionPulsesGroup(std::vector<PulseGroup> pulse_groups_stript, std::vector<PulseGroup> pulse_groups_latt, int shift_from_begin, std::vector<double> signal_abs_flt, int thDist, std::vector<UnionPulseGroup> & pulse_groups_un);

void fnGetCarInfo(QString dataFileName, std::vector<CarInfo> & cars_info, ProgressBarHelper *progressBarHelper = nullptr);

//функции получения времени
void fnGetSmplNum_for_Timestr(int Y,  int M, int D, int h, int m,  int s,  int ms, const int64_t * TSt, int NumTSt, int & NB, int & ND);
void fnGetTime_for_SmplNum(int ND, int & Y,  int & M, int & D, int & h, int & m,  int & s,  int & ms, const int64_t * TSt, int NumTSt);

//функция расчёта массы а/м
double fnGetCarWeit(double Amplitude, double Speed, size_t SizeGroup, double St = 0.58, double AmplitudeNorm = 1.5, double SpeedNorm = 20.0);
#endif // CARAXIS_H
