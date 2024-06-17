#ifndef SENSOR_DATA_PROCESSOR_H
#define SENSOR_DATA_PROCESSOR_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <unordered_map>

#include <cstring>
#include <cmath>
#include <QFile>
#include <QDebug>

#include <mutex>
#include <omp.h>

#include <complex>
#include <fftw3.h>
#include "CarAxis.h"

std::mutex fft_planer_mutex;

template<typename Iterator>
using iterator_value_t = typename std::iterator_traits<Iterator>::value_type;


template<typename Iterator>
using iterator_difference_t = typename std::iterator_traits<Iterator>::difference_type;


template<typename InputIterator, typename OutputIterator>
void rolling_mean(InputIterator begin, InputIterator end, OutputIterator result, iterator_difference_t<InputIterator> window_size)
{
    using value_type_input = iterator_value_t<InputIterator>;
    using difference_type_input = iterator_difference_t<InputIterator>;

    double acc = 0.0;

    for(difference_type_input i = 0; i < window_size; ++i, ++result)
    {
        acc += *(begin+i);
        *result = acc/(i+1);
    }

    std::transform(begin+window_size, end, begin, result, [&acc, window_size](const value_type_input &val, const value_type_input &prev)
    {
        acc += val - prev;
        return acc/window_size;
    });
}


template<typename InputIterator>
std::pair<InputIterator, double> rolling_variance_min(InputIterator begin, InputIterator end, iterator_difference_t<InputIterator> window_size)
{
    using value_type_input = iterator_value_t<InputIterator>;

    double acc_mean = std::accumulate(begin, begin+window_size, 0.0);
    double mean = acc_mean/window_size;

    double acc_var = std::accumulate(begin, begin+window_size, 0.0, [mean](double accumulator, const value_type_input &val)
    {
        return accumulator + (val - mean)*(val - mean);
    });

    auto min_var = acc_var/window_size;
    auto min_iter = begin;

    for(auto cur = begin+window_size, prev = begin; cur != end; ++cur, ++prev)
    {
        acc_mean += *cur - *prev;
        double mean_new = acc_mean/window_size;
        acc_var += (*cur - mean )*(*cur - mean_new) - (*prev - mean)*(*prev - mean_new);
        mean = mean_new;
        double var = acc_var/window_size;

        if(var < min_var)
        {
            min_var = var;
            min_iter = prev+1;
        }
    }

    return std::make_pair(min_iter, min_var);
}


template<typename InputIterator, typename OutputIterator>
void rolling_accumulate(InputIterator begin, InputIterator end, OutputIterator result, iterator_difference_t<InputIterator> window_size = 40)
{
    using value_type_input = iterator_value_t<InputIterator>;
    using difference_type_input = iterator_difference_t<InputIterator>;

    double acc = 0.0;

    for(difference_type_input i = 0; i < window_size; ++i, ++result)
    {
        acc += *(begin+i);
        *result = acc;
    }

    std::transform(begin+window_size, end, begin, result, [&acc](const value_type_input &val, const value_type_input &prev)
    {
        acc += val - prev;
        return acc;
    });
}


template<typename InputIterator>
std::pair<InputIterator, double> rolling_accumulate_min(InputIterator begin, InputIterator end, iterator_difference_t<InputIterator> window_size = 1000)
{
    double acc = std::accumulate(begin, begin+window_size, 0.0);

    double min_acc = acc;
    InputIterator min_iter = begin;

    for(auto cur = begin+window_size, prev = begin; cur != end; ++cur, ++prev)
    {
        acc += *cur - *prev;

        if(acc < min_acc)
        {
            min_acc = acc;
            min_iter = prev+1;
        }
    }

    return std::make_pair(min_iter, min_acc);
}


template<typename InputIterator, typename OutputIterator>
void mean_linear_approx(InputIterator begin,
                        InputIterator end,
                        OutputIterator result,
                        iterator_difference_t<InputIterator> win_size = 150)
{
    std::vector<double> res;
    res.reserve(std::distance(begin, end));
    rolling_mean(begin, end, std::back_inserter(res), win_size);

    for(uint i = 0; i < res.size(); ++i)
    {
        if(res[i] > *(begin+i)) res[i] = *(begin+i);
        else res[i] = 0.0;
    }

    auto it_sequence_begin = res.begin();
    auto it_sequence_end = res.end();

    auto it_gap_begin = std::find(it_sequence_begin, it_sequence_end, 0.0);

    do
    {
        auto it_gap_end = std::find_if(it_gap_begin, it_sequence_end, [](double val)
        {
            if(val > 0) return true;
            else return false;
        });

        double val_prev, val_next;

        if(it_gap_begin != it_sequence_begin) val_prev = *(it_gap_begin-1);
        else val_prev = 0.0;

        if(it_gap_end != it_sequence_end) val_next = *it_gap_end;
        else val_next = 0.0;

        auto dist = std::distance(it_gap_begin, it_gap_end);

        auto coef = (val_next - val_prev)/(dist+1);

        for(auto it = it_gap_begin; it != it_gap_end; ++it)
        {
            val_prev += coef;
            *it = val_prev;
        }

        it_gap_begin = std::find(it_gap_end, it_sequence_end, 0.0);

    }
    while(it_gap_begin != it_sequence_end);

    rolling_mean(it_sequence_begin, it_sequence_end, result, 10);
}


template<typename InputIterator>
double get_mean(InputIterator begin, InputIterator end)
{
    auto size = std::distance(begin, end);
    return std::accumulate(begin, end, 0.0)/size;
}

template<typename InputIterator>
double get_variance(InputIterator begin, InputIterator end)
{
    auto size = std::distance(begin, end);
    auto mean = get_mean(begin, end);

    double sq_sum = std::inner_product(begin, end, begin, 0.0);

    return sq_sum / size - mean * mean;
}

template<typename InputIterator>
double get_stdev(InputIterator begin, InputIterator end)
{
    return std::sqrt(get_variance(begin, end));
}

template<typename T>
void simple_filter(const T *data, size_t size, double *filtered, const std::vector<double> &coef)
{
    // add zero
    std::vector<T> tmp(size+coef.size()-1, 0);
    memcpy(tmp.data()+coef.size()-1, data, size*sizeof(T));

    //filter data
    for(size_t i = coef.size(), k = 0; i < size; i++, k++)
    {
        double val = 0.0;

        for(size_t j = 1; j <= coef.size(); j++)
            val += (coef[j - 1] * (tmp[i - j]));

        filtered[k] = val;
    }
}

template<typename InputIterator, typename OutputIterator>
void mean_linear_approx_new(InputIterator begin,
                            InputIterator end,
                            OutputIterator result,
                            size_t win_size,
                            double thres_low,
                            double thres_hight,
                            std::vector<std::tuple<iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>>> &max_pos,
                            double thres_step = 1.0,
                            int set_std_sign = 0)
{
    auto size = std::distance(begin, end);
    std::vector<double> tmp(size);

    std::copy(begin, end, tmp.begin());

    double stdev;
    if(set_std_sign == 0){
        stdev = get_stdev(begin, end);
    }
    else{
        stdev = 1.0 * set_std_sign;
    }

    for(auto &value : tmp) value /= stdev;

    std::vector<double> coef(2*win_size);
    for(size_t i = 0; i < coef.size(); ++i) {
        if(i < win_size) {
            coef[i] = -1.0/win_size;
        } else {
            coef[i] = 1.0/win_size;
        }
    }

    simple_filter(tmp.data(), tmp.size(), tmp.data(), coef);


//    QFile f("after_diff_filter.dat");
//    f.open(QFile::WriteOnly);
//    f.write((char *)tmp.data(), tmp.size()*sizeof(double));
//    f.close();



    std::copy(begin, end, result);

    auto it_sequence_begin = tmp.begin();
    auto it_sequence_end = tmp.end();

    auto it_current_low =  std::find_if(it_sequence_begin, it_sequence_end, [thres_low](double val){
        if(val <= thres_low) return true;
        else return false;
    });

    while(it_current_low != it_sequence_end) {

        auto it_current_hight = std::find_if(it_current_low, it_sequence_end, [thres_hight](double val){
            if(val >= thres_hight) return true;
            else return false;
        });

        auto it_current_hight_end = std::find_if(it_current_hight, it_sequence_end, [thres_hight](double val){
            if(val >= thres_hight) return false;
            else return true;
        });

        int64_t x0 = (int64_t)(std::distance(it_sequence_begin, it_current_low)) - win_size;
        if(x0 < 0) x0 = 0;
        int64_t x1 = std::distance(it_sequence_begin, it_current_hight_end) - win_size;

        auto max = std::max_element(begin+x0, begin+x1);
        auto pulse_center = std::distance(begin, max);

        if(it_current_hight_end == it_sequence_end) x1 = pulse_center + 1;

        max_pos.push_back(std::make_tuple(x0, pulse_center, x1));

        for(int64_t x = x0; x < x1; ++x) {
            *(result+x) = 0;
        }

        thres_low /= thres_step;

        it_current_low = std::find_if(it_current_hight_end, it_sequence_end, [thres_low](double val){
            if(val <= thres_low) return true;
            else return false;
        });       
    };

    for(size_t i = 0; i < tmp.size(); ++i) {
        *(result+i) = *(begin+i) - *(result+i);
    }
}

/*template<typename InputIterator, typename OutputIterator>
void mean_linear_approx_best(InputIterator begin,
                             InputIterator end,
                             OutputIterator result,
                             size_t win_size,
                             double thres_low,
                             double thres_hight)
{
    auto size = std::distance(begin, end);
    std::vector<double> tmp_result(size);

    std::copy(begin, end, tmp_result.begin());

    double mean = std::accumulate(begin, end, 0.0)/size;

    double sq_sum = std::inner_product(begin, end, begin, 0.0);
    double stdev = std::sqrt(sq_sum / size - mean * mean);

    for(auto &value : tmp_result) value /= stdev;

    std::vector<double> coef(2*win_size);
    for(size_t i = 0; i < coef.size(); ++i) {
        if(i < win_size) {
            coef[i] = -1.0/win_size;
        } else {
            coef[i] = 1.0/win_size;
        }
    }

    std::vector<double> tmp_filtered(size);
    simple_filter(tmp_result.data(), tmp_result.size(), tmp_filtered.data(), coef);

    std::copy(tmp_result.begin(), tmp_result.end(), result);

    auto it_sequence_begin = tmp_filtered.begin();
    auto it_sequence_end = tmp_filtered.end();

    auto it_current_low =  std::find_if(it_sequence_begin, it_sequence_end, [thres_low](double val){
        if(val <= thres_low) return true;
        else return false;
    });

    while(it_current_low != it_sequence_end) {

        auto it_current_hight = std::find_if(it_current_low, it_sequence_end, [thres_hight](double val){
            if(val >= thres_hight) return true;
            else return false;
        });

        if(it_current_hight != it_sequence_end) {
            auto x0 = std::distance(it_sequence_begin, it_current_low);
            auto x1 = std::distance(it_sequence_begin, it_current_hight);

            double y0 = tmp_result[x0];
            double y1 = tmp_result[x1];

            double a = (y1-y0)/(x1-x0);
            double b = y0 - a*x0;

            for(int x = x0+1; x < x1; ++x) {
                tmp_result[x] = a*x+b;
            }
        }

        it_current_low = std::find_if(it_current_hight, it_sequence_end, [thres_low](double val){
            if(val <= thres_low) return true;
            else return false;
        });

    };

    for(size_t i = 0; i < tmp_result.size(); ++i) {
        *(result+i) = *(result+i) - tmp_result[i];
    }
}*/

namespace SeismicDataProcessing
{
    template<class T>
    void autoCorrFunc(T* iBuffer, unsigned int uBuffSize, double *iCorr, unsigned int uOrderKF, const double *dbHammingWeitTable)
    {
        double iTemp;

        unsigned int i,j;

        for(i=0;i<uOrderKF;i++)
        {
            iTemp = 0.;

            for(j = i;j < uBuffSize-uOrderKF + i; j++)
            {
                iTemp += (dbHammingWeitTable[j-i] * iBuffer[j-i]) * (dbHammingWeitTable[j] * iBuffer[j]);
            }

            iCorr[i] = iTemp;
        }
    }

    inline void toeplForm(double * iVector, unsigned int uBuffSize, double ** iTMatrix, double * Z)
    {
        unsigned int i, uMatrOrdr = uBuffSize - 1;
        memcpy((void*)Z, (void*)(iVector+1), (size_t)(uMatrOrdr*sizeof(double)));
        double * iFullCFunc = new double[uBuffSize*2 - 1];
        for(i=0; i<uMatrOrdr; i++)
            iFullCFunc[i] = iVector[uMatrOrdr - i];
        memcpy((void*)&(iFullCFunc[uMatrOrdr]), (void*)iVector, (size_t)(uBuffSize*sizeof(double)));
        for(i=0; i<uMatrOrdr; i++)
            memcpy((void*)iTMatrix[i], (void*)&(iFullCFunc[uMatrOrdr - i]), (size_t)(uMatrOrdr*sizeof(double)));
        delete[] iFullCFunc;
    }

    template <typename T>
    void autoCorr(const T *data, int windowSize, int maxOffset, const double *hammingTable, double **correlationMatrix, double *correlationMatrixZero)
    {
        std::vector<std::vector<double> > result;
        for(int i = 0; i < maxOffset; ++i) result.emplace_back(std::vector<double>(maxOffset));

        std::unordered_map<int, std::vector<std::pair<int, int> > > indices_map;

        for(int k = -maxOffset+1; k<maxOffset; k++)
        {
            std::vector<std::pair<int, int> > vec;

            for(int j=0; j<maxOffset; j++)
            {
                for(int i=0; i<maxOffset; i++)
                {
                    if(i-j == k)
                    {
                        vec.push_back(std::make_pair(j, i));
                    }
                }
            }

            indices_map.insert(std::make_pair(k, vec));
        }

        for(int k = -maxOffset+1; k<maxOffset; k++)
        {
            double tmp = 0;
            for(int i = maxOffset-1; i < windowSize - maxOffset; i++)
            {
                tmp+=data[i]*hammingTable[i]*data[i+k]*hammingTable[i+k];
            }

            auto indices = indices_map[k];

            // foreach(auto index, indices)
            // {
            //     result[index.first][index.second] = tmp;
            //     if(index.second < maxOffset-1 && index.first < maxOffset-1) correlationMatrix[index.first][index.second]=tmp;
            //     if(index.first > 0 && index.second == 0) correlationMatrixZero[index.first-1]=tmp;
            // }

            for(auto index : indices)
            {
                result[index.first][index.second] = tmp;
                if(index.second < maxOffset-1 && index.first < maxOffset-1) correlationMatrix[index.first][index.second]=tmp;
                if(index.first > 0 && index.second == 0) correlationMatrixZero[index.first-1]=tmp;
            }
        }
    }

    inline int levinson(double **iInToeplitzMatrix, double *iInZMatrix, unsigned int uMatrixOrder, double *dbOutXMatrix)
    {
        double iToepl_0 = iInToeplitzMatrix[0][0];
        double *iToeplCol_0 = new double[uMatrixOrder-1];
        double *iToeplRaw_0 = new double[uMatrixOrder-1];
        double *dbTempCol = new double[uMatrixOrder];
        double *dbTempRaw = new double[uMatrixOrder];

        memset((void*)dbOutXMatrix, 0, (size_t)(uMatrixOrder * sizeof(double)));
        memset((void*)dbTempCol, 0, (size_t)(uMatrixOrder * sizeof(double)));
        memset((void*)dbTempRaw, 0, (size_t)(uMatrixOrder * sizeof(double)));

        for(unsigned int i = 0; i<uMatrixOrder-1; i++)
        {
            iToeplCol_0[i] = iInToeplitzMatrix[i+1][0];
            iToeplRaw_0[i] = iInToeplitzMatrix[0][i+1];
        }
        double dbP = iToepl_0;
        if(dbP == 0)
        {
            delete[] dbTempCol;
            delete[] dbTempRaw;
            delete[] iToeplCol_0;
            delete[] iToeplRaw_0;

            return -1;
        }
        dbOutXMatrix[0] = 1. * iInZMatrix[0] / iToepl_0;
        if(uMatrixOrder == 1)
        {
            delete[] dbTempCol;
            delete[] dbTempRaw;
            delete[] iToeplCol_0;
            delete[] iToeplRaw_0;

            return 0;
        }

        for(unsigned int i = 0; i<uMatrixOrder-1; i++)
        {
            double dbSave1 = iToeplCol_0[i];
            double dbSave2 = iToeplRaw_0[i];
            double dbBeta = dbOutXMatrix[0] * iToeplCol_0[i];
            if(i)
            {
                for(unsigned int j = 0; j<=(i-1); j++)
                {
                    dbSave1 += dbTempCol[j] * iToeplCol_0[i-j-1];
                    dbSave2 += dbTempRaw[j] * iToeplRaw_0[i-j-1];
                    dbBeta += dbOutXMatrix[j+1] * iToeplCol_0[i-j-1];
                }
            }
            double dbTemp1 = -dbSave1 / dbP;
            double dbTemp2 = -dbSave2 / dbP;
            dbP = dbP * (1 - dbTemp1 * dbTemp2);
            if(dbP == 0)
            {
                delete[] dbTempCol;
                delete[] dbTempRaw;
                delete[] iToeplCol_0;
                delete[] iToeplRaw_0;

                return -1;
            }

            dbTempCol[i] = dbTemp1;
            dbTempRaw[i] = dbTemp2;
            double dbAlfa = (iInZMatrix[i+1] - dbBeta) / dbP;
            if(i)
            {
                for(unsigned int j = 0; j<=(i-1); j++)
                {
                    dbSave1 = dbTempCol[j];
                    dbTempCol[j] = dbSave1 + dbTemp1 * dbTempRaw[i-j-1];
                    dbTempRaw[i-j-1] = dbTempRaw[i-j-1] + dbTemp2 * dbSave1;
                }

            }
            dbOutXMatrix[i+1] = dbAlfa;
            for(unsigned int j = 0; j<=i; j++)
                dbOutXMatrix[j] = dbOutXMatrix[j] + dbAlfa * dbTempRaw[i-j];
        }
        delete[] dbTempCol;
        delete[] dbTempRaw;
        delete[] iToeplCol_0;
        delete[] iToeplRaw_0;
        return 0;
    }

    template<typename Ti>
    void bleachCoeffCalc(Ti *iInBuffer, unsigned int uLenInBuffer, unsigned int uNumCoeff, const double *dbHammingWeitTable, std::vector<double> &lpc)
    {
        lpc.clear();

        double *iCorr = new double[uNumCoeff + 1];
        double *iZ = new double[uNumCoeff];
        double **T = new double*[uNumCoeff];

        for(unsigned int i = 0; i<uNumCoeff; i++)
            T[i] = new double[uNumCoeff];

        double *dbOutXMatrix = new double[uNumCoeff];

        autoCorrFunc(iInBuffer, uLenInBuffer, iCorr, uNumCoeff + 1, dbHammingWeitTable);
        toeplForm(iCorr, uNumCoeff + 1, T, iZ);
        autoCorr(iInBuffer, uLenInBuffer, uNumCoeff + 1, dbHammingWeitTable, T, iZ);
        levinson(T, iZ, uNumCoeff, dbOutXMatrix);

        for(unsigned int n = 0; n < uNumCoeff; n++)
            lpc.push_back(dbOutXMatrix[n]);

        delete[] dbOutXMatrix;

        for(unsigned int i = 0; i < uNumCoeff; i++)
            delete[] T[i];

        delete[] T;
        delete[] iZ;
        delete[] iCorr;
    }
};


template<typename T>
class passband_filter
{
    public:
        passband_filter()
        {

        }

        void filter(const T *data, size_t size, double *filtered)
        {
            size_t offset = m_b.size()-1;

            for(size_t i = 0; i < size; ++i) {

                double b_sum = 0.0;
                for(size_t j = i, k = 0; k < m_b.size()-offset; --j, ++k) {
                    b_sum += data[j]*m_b[k];
                }

                double a_sum = 0.0;
                for(size_t j = i-1, k = 0; k < m_a.size()-offset; --j, ++k) {
                    a_sum += filtered[j]*m_a[k];
                }

                filtered[i] = b_sum - a_sum;

                if(offset > 0) --offset;
            }
        }

        void set_b(const std::vector<double> &b) {
            m_b.insert(m_b.end(), b.begin(), b.end());
        }

        void set_a(const std::vector<double> &a) {
            m_a.insert(m_a.end(), a.begin(), a.end());
        }

        std::vector<double> &b() {
            return m_b;
        }

        std::vector<double> &a() {
            return m_a;
        }

    private:
        std::vector<double> m_b;
        std::vector<double> m_a;
};

void create_hamming_window(std::vector<double> &hamming_window, size_t hamming_window_size, double hamming_window_pow)
{
    hamming_window.clear();
    hamming_window.reserve(hamming_window_size);

    double a = 0.53836;
    double b = 0.46164;

    for(size_t n = 0; n < hamming_window_size; n++)
    {
        double value = pow(a - b * cos((2 * M_PI * n) / (hamming_window_size - 1)), hamming_window_pow);
        hamming_window.push_back(value);
    }
}

//typedef struct CarAxisDetect
//{
//    int    coord_ax_signal;
//    double  pow_ax_signal;
//    double  pow_ax_signal_approx;
//} CarAxisDetect;

//typedef struct CrackDetect
//{
//    int    coord_crack_signal;
//    double  pow_crack_signal;
//    double  pow_crack_signal_approx;
//} CrackDetect;


template<typename InputIterator, typename OutputIterator>
void remove_crack(InputIterator begin,
                  InputIterator end,
                  OutputIterator result,
                  const std::vector<double> &coef,
                  const std::vector<double> &coef_passband_b,
                  const std::vector<double> &coef_passband_a,
                  const std::vector<double> &hamming_weight,
                  std::vector<CrackDetect> &cracks,
                  size_t crack_max_count = 5,
                  double crack_stdev_mult = 20.0,
                  double crack_filter_thres = 0.4)
{
    auto size = std::distance(begin, end);

    std::copy(begin, end, result);

    std::vector<double> tmp_filtered(size);
    std::copy(begin, end, tmp_filtered.begin());

    simple_filter(tmp_filtered.data(), size, tmp_filtered.data(), coef);

    double stdev_base = get_stdev(begin, end);

    double crack_threshold = crack_stdev_mult * stdev_base;

    using value_type_input = iterator_value_t<InputIterator>;
    using difference_type_input = iterator_difference_t<InputIterator>;

    std::vector<std::pair<difference_type_input, value_type_input>> crack_points;

    difference_type_input crack_window = coef.size();

    for(size_t i = 0; i < tmp_filtered.size(); i += crack_window) {
        auto it = tmp_filtered.begin()+i;
        std::vector<double>::iterator it_stop;
        if(std::distance(tmp_filtered.begin(), it) + crack_window > tmp_filtered.size()-1) {
            it_stop = tmp_filtered.end();
        } else {
            it_stop = it + crack_window;
        }
        auto it_max = std::max_element(it, it_stop);
        auto dist = std::distance(tmp_filtered.begin(), it_max);

        crack_points.emplace_back(dist, tmp_filtered[dist]);
    }

    size_t i = 1;
    while(i < crack_points.size()) {
        if(crack_points[i].first - crack_points[i-1].first <= crack_window/2) {
            if(crack_points[i].second > crack_points[i-1].second) crack_points.erase(crack_points.begin()+i-1);
            else crack_points.erase(crack_points.begin()+i);
        } else {
            ++i;
        }
    }

    std::sort(crack_points.begin(), crack_points.end(), [](std::pair<difference_type_input, value_type_input> element1,
                                                           std::pair<difference_type_input, value_type_input> element2) {
        if(element2.second < element1.second) return true;
        else return false;
    });
    
    auto found = std::find_if(crack_points.begin(), std::min(crack_points.begin()+crack_max_count, crack_points.end()), [crack_threshold](std::pair<difference_type_input, value_type_input> element){
        if(element.second < crack_threshold) return true;
        else return false;
    });

    crack_points.erase(found, crack_points.end());

    std::sort(crack_points.begin(), crack_points.end(), [](std::pair<difference_type_input, value_type_input> element1,
                                                           std::pair<difference_type_input, value_type_input> element2) {
        if(element2.first > element1.first) return true;
        else return false;
    });

    std::vector<double> tmp_crack_res(size);
    std::copy(begin, end, tmp_crack_res.begin());

    double stdev_coef = get_stdev(coef.begin(), coef.end());

    double mean_coef = get_mean(coef.begin(), coef.end());

    std::vector<std::pair<double, std::pair<difference_type_input, difference_type_input>>> ranges;

    for(size_t i = 0; i < crack_points.size(); ++i) {

        difference_type_input win_end = crack_points[i].first + 1;
        difference_type_input win_begin = win_end - crack_window;
        if(win_begin < 0) win_begin = 0;

        difference_type_input win_size = win_end-win_begin;

        double stdev_crack = get_stdev(begin+win_begin, begin+win_end);

        int startIndex = coef.size()-1;

        if(win_size < coef.size()) {
            startIndex = coef.size() - (coef.size() - win_size) - 1;
        }

        for(int j = startIndex, k = 0; j >= 0; --j, ++k) {
            tmp_crack_res[win_begin+k] = (coef[j]-mean_coef)*stdev_crack/stdev_coef;
        }

        for(auto i = win_begin; i < win_end; ++i) {
            tmp_crack_res[i] = *(begin+i) - tmp_crack_res[i];
        }

        double stdev_res = get_stdev(tmp_crack_res.begin()+win_begin, tmp_crack_res.begin()+win_end);

        ranges.push_back(std::make_pair(stdev_res/stdev_crack, std::make_pair(win_begin, win_end)));
    }

    passband_filter<value_type_input> f;
    f.set_b(coef_passband_b);
    f.set_a(coef_passband_a);

    std::vector<double> tmp_crack_final(tmp_crack_res.size());

    f.filter(tmp_crack_res.data(), tmp_crack_res.size(), tmp_crack_final.data());

    for(auto elem : ranges) {
        auto win_begin = elem.second.first;
        auto win_end = elem.second.second;

        if(elem.first < crack_filter_thres) {
            for(auto i = win_begin, k = 0; i < win_end; ++i, ++k) {
                *(result+i) = tmp_crack_final[i]*hamming_weight[k] + (1.0 - hamming_weight[k]) * tmp_crack_res[i];
            }
        }

        cracks.push_back({ win_begin+crack_window/2, elem.first });
    }
}

static std::vector<double> coef_noise_strip =
{
    0.00183821, 0.000269771, -0.00141273, -0.00308496, -0.00504312, -0.00712255, -0.00950255, -0.0118441, -0.0136689, -0.015526, -0.0178442, -0.0206657, -0.0234725, -0.026509, -0.0301109, -0.0337904,
    -0.036555, -0.0383653, -0.0401051, -0.0420168, -0.0450169, -0.0495526, -0.0550921, -0.060238, -0.0647654, -0.0682183, -0.0708926, -0.0735022, -0.0762221, -0.0784619, -0.0804571, -0.0822219, -0.0838365,
    -0.0859625, -0.0886191, -0.0912386, -0.0930865, -0.0941868, -0.0951635, -0.0966486, -0.0987781, -0.101708, -0.104201, -0.105443, -0.10625, -0.107447, -0.108502, -0.109662, -0.110589, -0.111083, -0.11052,
    -0.109047, -0.10769, -0.106951, -0.1064, -0.104651, -0.102064, -0.0994174, -0.0959476, -0.0919072, -0.0880171, -0.0835121, -0.0787246, -0.0740554, -0.0692098, -0.0639925, -0.0585024, -0.0519254, -0.0441825,
    -0.0363333, -0.0290319, -0.0223837, -0.0163103, -0.0107889, -0.00503974, 0.00163166, 0.00938624, 0.0177025, 0.0262862, 0.0352204, 0.0442353, 0.0527412, 0.0610509, 0.0685933, 0.0759618, 0.0829007, 0.0891075,
    0.0953577, 0.102049, 0.109164, 0.115648, 0.120776, 0.124427, 0.126903, 0.128919, 0.130368, 0.131185, 0.132118, 0.133151, 0.134645, 0.136509, 0.138003, 0.138086, 0.136219, 0.133376, 0.130955, 0.129347,
    0.128143, 0.126406, 0.12377, 0.12094, 0.117601, 0.11426, 0.111081, 0.107521, 0.103225, 0.0973388, 0.089389, 0.0790558, 0.0669854, 0.0542238, 0.0415341, 0.029195, 0.0165345, 0.00373112, -0.00826847, -0.0191306,
    -0.0286461, -0.0369371, -0.0442916, -0.0506817, -0.0562064, -0.0615572, -0.0663093, -0.0701411, -0.0724376, -0.073659, -0.0739641, -0.0734751, -0.0723536, -0.0704765, -0.0680127, -0.0652837, -0.0619966,
    -0.0582496, -0.0543078, -0.0503737, -0.0459895, -0.0405808, -0.0339022, -0.0266734, -0.0206915, -0.0168323, -0.0147606, -0.0135667, -0.0119155, -0.0092936, -0.00665108, -0.00488796, -0.00381327, -0.0030376,
};

static std::vector<double> coef_noise_latt =
{
    -0.000338758150528659,	-0.00122998946436122,	-0.00163029898258950,	-0.00238358122673163,	-0.00295703781084222,	-0.00345111840097653,	-0.00397015346789815,	-0.00412691992786040,	-0.00446843074219380,
    -0.00480872316435134,	-0.00498381063258781,	-0.00592571803580943,	-0.00639520515424558,	-0.00690593710411682,	-0.00712348779596425,	-0.00686234573960227,	-0.00694821982518320,	-0.00738679588286111,
    -0.00794256321760348,	-0.00829607745115104,	-0.00884290990993701,	-0.00895409947739440,	-0.00909805025669191,	-0.00961528029816780,	-0.0101020054096107,	-0.0103561529923705,	-0.0116404285970122,
    -0.0116845614691605,	-0.0122978188643496,	-0.0119581130754619,	-0.0115027051555025,	-0.0120536891728137,	-0.0119616780007172,	-0.0122433973471606,	-0.0125427608173361,	-0.0124915883459494,
    -0.0123819781757571,	-0.0124939800046650,	-0.0126802135050289,	-0.0129419873202969,	-0.0126229490727629,	-0.0124368960749436,	-0.0124302626064305,	-0.0125509285574780,	-0.0116387138228388,
    -0.0111695425838556,	-0.0107172483327901,	-0.0101354886316292,	-0.0102581401106670,	-0.0100756069124668,	-0.00991514015034083,	-0.00921930284100928,	-0.00916885237980094,	-0.00834607665599795,
    -0.00759202727604138,	-0.00823637623453329,	-0.00797157900164370,	-0.00820677381722321,	-0.00798701196920475,	-0.00739212070792603,	-0.00727434279759170,	-0.00732380049480489,	-0.00729519084148996,
    -0.00752948314434660,	-0.00784175254645908,	-0.00791120090048380,	-0.00787352099430404,	-0.00806868937062434,	-0.00797203025800513,	-0.00824892116138116,	-0.00900112039025584,	-0.00918311208082226,
    -0.0100425298211737,	-0.00973716464139118,	-0.0104193288829713,	-0.0110185973309559,	-0.0116080283902613,	-0.0125110374951272,	-0.0126548528975163,	-0.0131564694688866,	-0.0136618314680568,
    -0.0140926007906819,	-0.0152170413921036,	-0.0169041084249615,	-0.0170603333772901,	-0.0172339316995338,	-0.0180233144525907,	-0.0189252856678254,	-0.0185780438977017,	-0.0194023538931336,
    -0.0201738217686415,	-0.0211791306906446,	-0.0212039046648874,	-0.0218534430715358,	-0.0217399972222712,	-0.0221419312634007,	-0.0217898610502097,	-0.0220598477312558,	-0.0216502423319820,
    -0.0210366690573399,	-0.0202348767543436,	-0.0202525660037118,	-0.0196189118209859,	-0.0187866597135928,	-0.0181368054274914,	-0.0165054234296344,	-0.0155183001389971,	-0.0141888537725758,
    -0.0127158176319461,	-0.0112441352604007,	-0.00956397244987273,	-0.00802879830827355,	-0.00624087547863508,	-0.00451662492159498,	-0.00281768984643133,	-0.00115647980308359,	QString("-7.17497614680389e-06").toDouble(),
    0.00183927580356968,	0.00323131142732193,	0.00418612476248057,	0.00544282860343869,	0.00675941416355913,	0.00755845380275068,	0.00814427481116457,	0.00888347785682994,	0.00939244990689159,
    0.00958166169924095,	0.00944601403699383,	0.00904349336259451,	0.00774865835909520,	0.00666221354330620,	0.00513331186513097,	0.00335572280616934,	0.00190087229690545,	-0.000825573513243882,
    -0.00264720519308149,	-0.00512013517937709,	-0.00772189885686215,	-0.0101133319442827,	-0.0128932516332620,	-0.0155330110963798,	-0.0183906822564342,	-0.0218143642706356,	-0.0250587621324391,
    -0.0282692255158628,	-0.0310950831024361,	-0.0346381675498729,	-0.0374140259315993,	-0.0409270567053646,	-0.0445142740249496,	-0.0466394658591240,	-0.0496786774533834,	-0.0517230943988611,
    -0.0535104305952297,	-0.0555616615117650,	-0.0576096433824980,	-0.0599809504361986,	-0.0615707717231675,	-0.0629588362909391,	-0.0643436969385446,	-0.0653149811308956,	-0.0662130264157858,
    -0.0666005653789855,	-0.0667988925498358,	-0.0668414008990829,	-0.0665543567275746,	-0.0658878059561001,	-0.0650256806775801,	-0.0639716360685421,	-0.0628045517409648,	-0.0613029059470202,
    -0.0594275747601719,	-0.0571101928416627,	-0.0545713343009615,	-0.0513559973488344,	-0.0485786949467514,	-0.0450233363262836,	-0.0418155353553924,	-0.0380494851397709,	-0.0336194111882990,
    -0.0292048153300243,	-0.0249373290455771,	-0.0205988601354764,	-0.0160505570173087,	-0.0117749481183558,	-0.00659515684799708,	-0.00194297451542727,	0.00361591722417230,	0.00863605399385557,
    0.0138495992400495,	0.0189918911067730,	0.0240183454655164,	0.0288120869186681,	0.0334103892416827,	0.0381963239597816,	0.0423052387588208,	0.0462310337263899,	0.0494067052443470,	0.0524473157333269,	0.0555366619093416,
    0.0584898640411116,	0.0609529566387279,	0.0629259397021906,	0.0642779940123198,	0.0656440823952895,	0.0666773692117016,	0.0669963172079633,	0.0666936144407132,	0.0667774127470315,	0.0663019690446245,	0.0647024006202486,
    0.0628961567823359,	0.0609267837697647,	0.0586919817653980,	0.0570185878002915,	0.0535190044660970,	0.0503744696370788,	0.0474019988586756,	0.0443101707726729,	0.0400862307271042,	0.0363271749850850,	0.0326622513200609,
    0.0288661925564041,	0.0245095831405737,	0.0210696107717246,	0.0171051431339805,	0.0137853403341813,	0.0107426991915750,	0.00743300453427197,	0.00455019839488568,	0.00331231194419937,	0.00197663823999026,
    0.00216539877597819,	0.00191874204881825,	0.00166549697878137,	0.00250636808267787,	0.00366397602666504,	0.00483634005367111,	0.00637602675888463,	0.00832012941521548,	0.0104261879796651,
    0.0131022735798783,	0.0162803818821872,	0.0199620020325843,	0.0236915907342019,	0.0277368783862788,	0.0320659611640618,	0.0369920561080221,	0.0412096334388947,	0.0452814098393878,	0.0499194227222086,	0.0543005804841010,
    0.0572786919670222,	0.0612368420157062,	0.0647149906727326,	0.0683135345269897,	0.0713460675270999,	0.0745465130192997,	0.0778985806489414,	0.0797559518326046,	0.0822208494556667,	0.0836128850794189,	0.0854217462042280,
    0.0874549719919421,	0.0880456214434233,	0.0893024155356537,	0.0891144221354802,	0.0883548674279141,	0.0874121026376058,	0.0863726787346782,	0.0849501833065292,	0.0831368547437420,	0.0814201401679378,	0.0791498242879262,
    0.0767306389342774,	0.0741741362698442,	0.0704275352034006,	0.0672732983626116,	0.0635837812258894,	0.0592450866876079,	0.0545276526851747,	0.0508963024934413,	0.0455880385369823,	0.0402478707557699,	0.0342996801531611,
    0.0282129087219558,	0.0217927039652867,	0.0148705667577944,	0.00795912432606811,	0.000443855757106686,	-0.00696848123581160,	-0.0147373107542629,	-0.0228131299753979,	-0.0306093958806244,	-0.0380359023232917,
    -0.0456039227609048,	-0.0531359780665116,	-0.0606222759770689,	-0.0681390336920231,	-0.0754213637272122,	-0.0817568225392039,	-0.0889301290374705,	-0.0949688867918192,	-0.100088525589199,	-0.105960183112908,
    -0.110976213449703,	-0.115485798772059,	-0.118951267125329,	-0.121874505834700,	-0.124343239136834,	-0.126550649880062,	-0.127952387515585,	-0.128910134017093,	-0.128899439241327,	-0.128463029214184,	-0.127543413875217,
    -0.126120963572704,	-0.124324015615837,	-0.122491598908961,	-0.119727969574630,	-0.116422832606577,	-0.114086949177249,	-0.110497385324585,	-0.107505781276457,	-0.103693612660697,	-0.0994934538253754,	-0.0957362031088019,
    -0.0911461136515654,	-0.0872158963716542,	-0.0827218342681309,	-0.0786646334481121,	-0.0747467354668681,	-0.0703301992062392,	-0.0662553091368522,	-0.0631951592474222,	-0.0590421115762348,	-0.0548439382689039,
    -0.0514184512292569,	-0.0477229323829276,	-0.0435961027063395,	-0.0390681061244363,	-0.0346858202216403,	-0.0298516010728679,	-0.0253383605739837,	-0.0209127991861262,	-0.0155604023575189,	-0.0117217901189788,
    -0.00700367923200347,	-0.00247866094208581,	0.00248633230023019,	0.00727023636470265,	0.0120905568175429,	0.0178863581468954,	0.0232691245286272,	0.0279880928026892,	0.0328839084447974,	0.0376489048677258,	0.0423883601805969,
    0.0466180311819559,	0.0508833965615043,	0.0543274753632425,	0.0579828323934041,	0.0612041710551384,	0.0638160428751196,	0.0666379745313484,	0.0692556224327559,	0.0710942664774196,	0.0733747807511996,	0.0745651950326631,	0.0754050733725644,
    0.0760094410174333,	0.0758714919477428,	0.0750059371208759,	0.0735602471157491,	0.0721403689744964,	0.0702142714469867,	0.0683602395603982,	0.0660554928200091,	0.0630830671672420,	0.0600567163792835,	0.0567470217219804,	0.0534447727946410,
    0.0497275033916906,	0.0469441992800006,	0.0432656025472250,	0.0394900308223773,	0.0355633784677214,	0.0313594742046004,	0.0277265897412381,	0.0239718209346525,	0.0198384029151875,	0.0159803866531057,	0.0119688981025002,	0.00802031468867859,
    0.00443950520942587,	0.00123098222835637,	-0.00125101801080405,	-0.00417303832799897,	-0.00696500656182855,	-0.00943161895906407,	-0.0122530091076592,	-0.0146970135611868,	-0.0167620077967459,	-0.0184503383474159,
    -0.0199103331792001,	-0.0210897368054445,	-0.0222715772160407,	-0.0225414736458145,	-0.0235328387462493,	-0.0240870718093628,	-0.0239670376172213,	-0.0235716016676965,	-0.0237936649231583,	-0.0238001630147629,
    -0.0232013006975036,	-0.0227398008166648,	-0.0217368384277412,	-0.0207326125210055,	-0.0200956641668411,	-0.0194177868606946,	-0.0184799858903621,	-0.0180862195893746,	-0.0175018426013173,	-0.0156183436743271,
    -0.0148170928789645,	-0.0131039883540518,	-0.0128839106265803,	-0.0129655880279999,	-0.0120459275633970,	-0.0115410168205883,	-0.0106648574692276,	-0.0104857538193744,	-0.00978490756443091,	-0.00916357268037215,
    -0.00927692827836443,	-0.00870369732243456,	-0.00801268845617035,	-0.00716103232523557,	-0.00660296358304989,	-0.00630928594302851,	-0.00543930880381950,	-0.00525095439855686,	-0.00498074208933006,	-0.00455597447631204,
    -0.00456698513153104,	-0.00431301805131586,	-0.00450714853800486,	-0.00460141599190847,	-0.00492262026997733,	-0.00471450083608388,	-0.00474536677120598,	-0.00417466285090014,	-0.00419384124626109,	-0.00404695730061426,
    -0.00394411597584340,	-0.00402367247236425,	-0.00425160206052467,	-0.00461698433637795,	-0.00459676805138569,	-0.00431035563878340,	-0.00456743638789247,	-0.00441811565789389,	-0.00408964615240593,	-0.00472027691751024,
    -0.00458160583764150,	-0.00462474594579461,	-0.00454415155964247,	-0.00445773596642781,	-0.00439753836781249,	-0.00480989643089107,	-0.00466441137996468,	-0.00439058901984640,	-0.00475858858259600,	-0.00532906687472112,
    -0.00564499145336121,	-0.00606226821077942,	-0.00610184339367720,	-0.00635183941791175,	-0.00628500835078335,	-0.00609259263826780,	-0.00574702051668149,	-0.00655792819817876,	-0.00653423723920346,	-0.00595459844294122,
    -0.00575812142317277,	-0.00629719227254207,	-0.00594065462137291,	-0.00615567827759630,	-0.00570248151380793,	-0.00530790295136986,	-0.00452903447153442,	-0.00414736184103337,	-0.00367471592806718,	-0.00368437281420187,
    -0.00302098083725744,	-0.00273240239412027,	-0.00270054369500301,	-0.00250077250379609,	-0.00177659629496647,	-0.00150498509101922,	-0.00141838899526000
};


static std::vector<double> coef_noise_strip_hamming_weight =
{
    0.531829589694499, 0.532418018074020, 0.534170798527590, 0.537051379106381, 0.541001887640206, 0.545947190595125, 0.551799692251562, 0.558464265244545,
    0.565842804462912, 0.573838063884118, 0.582356614660018, 0.591310912362214, 0.600620563063497, 0.610212931365832, 0.620023248580198, 0.629994369323896,
    0.640076301840126, 0.650225610135336, 0.660404759945533, 0.670581458250394, 0.680728018412872, 0.690820769858772, 0.700839521871560, 0.710767084766201,
    0.720588847664505, 0.730292409682178, 0.739867260049788, 0.749304502147958, 0.758596616369101, 0.767737256933874, 0.776721078162053, 0.785543586140572,
    0.794201012193083, 0.802690205003771, 0.811008538665585, 0.819153834300968, 0.827124293238841, 0.834918440025585, 0.842535073802768, 0.849973226803806,
    0.857232128909640, 0.864311177363748, 0.871209910882954, 0.877927987516108, 0.884465165700562, 0.890821288049300, 0.896996267471650, 0.902990075289894,
    0.908802731064290, 0.914434293881520, 0.919884854897638, 0.925154530957078, 0.930243459135238, 0.935151792074135, 0.939879693999342, 0.944427337322350,
    0.948794899746072, 0.952982561802755, 0.956990504763493, 0.960818908866955, 0.964467951822196, 0.967937807546633, 0.971228645105566, 0.974340627824232,
    0.977273912547313, 0.980028649024188, 0.982604979401191, 0.985003037804635, 0.987222950000574, 0.989264833119172, 0.991128795433201, 0.992814936181626,
    0.994323345430516, 0.995654103964589, 0.996807283203713, 0.997782945139511, 0.998581142288001, 0.999201917654898, 0.999645304710807, 0.999911327374129,
    1, 0.999911327374129, 0.999645304710807, 0.999201917654898, 0.998581142288001, 0.997782945139511, 0.996807283203713, 0.995654103964589, 0.994323345430516,
    0.992814936181626, 0.991128795433201, 0.989264833119172, 0.987222950000574, 0.985003037804635, 0.982604979401191, 0.980028649024188, 0.977273912547313,
    0.974340627824232, 0.971228645105566, 0.967937807546633, 0.964467951822196, 0.960818908866955, 0.956990504763493, 0.952982561802755, 0.948794899746072,
    0.944427337322350, 0.939879693999342, 0.935151792074135, 0.930243459135238, 0.925154530957078, 0.919884854897638, 0.914434293881520, 0.908802731064290,
    0.902990075289894, 0.896996267471650, 0.890821288049300, 0.884465165700562, 0.877927987516108, 0.871209910882954, 0.864311177363748, 0.857232128909640,
    0.849973226803806, 0.842535073802768, 0.834918440025585, 0.827124293238841, 0.819153834300968, 0.811008538665585, 0.802690205003771, 0.794201012193083,
    0.785543586140572, 0.776721078162053, 0.767737256933874, 0.758596616369101, 0.749304502147958, 0.739867260049788, 0.730292409682178, 0.720588847664505,
    0.710767084766201, 0.700839521871560, 0.690820769858772, 0.680728018412872, 0.670581458250394, 0.660404759945533, 0.650225610135336, 0.640076301840126,
    0.629994369323896, 0.620023248580198, 0.610212931365832, 0.600620563063497, 0.591310912362214, 0.582356614660018, 0.573838063884118, 0.565842804462912,
    0.558464265244545, 0.551799692251562, 0.545947190595125, 0.541001887640206, 0.537051379106381, 0.534170798527590, 0.532418018074020, 0.531829589694499
};

static std::vector<double> coef_noise_strip_hamming_weight_lat =
{
    0.531829589694499,  0.531889941524406,  0.532070864339646,  0.532371961190630,  0.532792574055059,  0.533331789111999,  0.533988443994819,  0.534761136897070,
    0.535648237376060,  0.536647898674164,  0.537758071358266,  0.538976518063627,  0.540300829119937,  0.541728438834321,  0.543256642208280,  0.544882611872555,
    0.546603415035091,  0.548416030251989,  0.550317363848809,  0.552304265839106,  0.554373545207901,  0.556521984449196,  0.558746353268044,  0.561043421378528,
    0.563409970348817,  0.565842804462912,  0.568338760585497,  0.570894717031323,  0.573507601453594,  0.576174397776989,  0.578892152210165,  0.581657978380002,
    0.584469061635520,  0.587322662573541,  0.590216119840874,  0.593146852269253,  0.596112360399685,  0.599110227452335,  0.602138119796820,  0.605193786975923,
    0.608275061333410,  0.611379857293950,  0.614506170340226,  0.617652075729228,  0.620815726986607,  0.623995354214764,  0.627189262247287,  0.630395828679254,
    0.633613501800058,  0.636840798452607,  0.640076301840126,  0.643318659299357,  0.646566580056669,  0.649818832981466,  0.653074244349389,  0.656331695626009,
    0.659590121280139,  0.662848506634439,  0.666105885759690,  0.669361339417963,  0.672613993058874,  0.675863014872208,  0.679107613899383,  0.682347038205542,
    0.685580573113424,  0.688807539499643,  0.692027292153560,  0.695239218198521,  0.698442735574913,  0.701637291584216,  0.704822361492980,  0.707997447195494,
    0.711162075933727,  0.714315799073024,  0.717458190931938,  0.720588847664505,  0.723707386193228,  0.726813443191006,  0.729906674110221,  0.732986752257195,
    0.736053367910264,  0.739106227479687,  0.742145052707687,  0.745169579906912,  0.748179559235677,  0.751174754008356,  0.754154940039375,  0.757119905019274,
    0.760069447921373,  0.763003378437633,  0.765921516442330,  0.768823691482235,  0.771709742292047,  0.774579516333846,  0.777432869359426,  0.780269664994384,
    0.783089774342900,  0.785893075612205,  0.788679453755749,  0.791448800134157,  0.794201012193083,  0.796935993157121,  0.799653651738972,  0.802353901863104,
    0.805036662403180,  0.807701856932556,  0.810349413487201,  0.812979264340408,  0.815591345788703,  0.818185597948391,  0.820761964562199,  0.823320392815505,
    0.825860833161682,  0.828383239156077,  0.830887567298213,  0.833373776881781,  0.835841829852038,  0.838291690670240,  0.840723326184753,  0.843136705508510,
    0.845531799902485,  0.847908582664905,  0.850267029025881,  0.852607116047223,  0.854928822527143,  0.857232128909640,  0.859517017198296,  0.861783470874297,
    0.864031474818446,  0.866261015236983,  0.868472079591014,  0.870664656529384,  0.872838735824803,  0.874994308313094,  0.877131365835374,  0.879249901183061,
    0.881349908045532,  0.883431380960336,  0.885494315265815,  0.887538707056014,  0.889564553137791,  0.891571850989997,  0.893560598724629,  0.895530795049877,
    0.897482439234952,  0.899415531076618,  0.901330070867355,  0.903226059365053,  0.905103497764185,  0.906962387668373,  0.908802731064290,  0.910624530296824,
    0.912427788045454,  0.914212507301773,  0.915978691348104,  0.917726343737163,  0.919455468272707,  0.921166068991134,  0.922858150143981,  0.924531716181277,
    0.926186771735715,  0.927823321607603,  0.929441370750553,  0.931040924257880,  0.932621987349668,  0.934184565360483,  0.935728663727692,  0.937254287980364,
    0.938761443728727,  0.940250136654151,  0.941720372499633,  0.943172157060767,  0.944605496177160,  0.946020395724292,  0.947416861605786,  0.948794899746072,
    0.950154516083430,  0.951495716563390,  0.952818507132468,  0.954122893732236,  0.955408882293693,  0.956676478731935,  0.957925688941105,  0.959156518789610,
    0.960368974115589,  0.961563060722630,  0.962738784375712,  0.963896150797367,  0.965035165664051,  0.966155834602719,  0.967258163187578,  0.968342156937030,
    0.969407821310782,  0.970455161707115,  0.971484183460316,  0.972494891838247,  0.973487292040064,  0.974461389194057,  0.975417188355627,  0.976354694505376,
    0.977273912547313,  0.978174847307163,  0.979057503530790,  0.979921885882700,  0.980767998944654,  0.981595847214352,  0.982405435104210,  0.983196766940212,
    0.983969846960834,  0.984724679316042,  0.985461268066350,  0.986179617181949,  0.986879730541886,  0.987561611933306,  0.988225265050742,  0.988870693495458,
    0.989497900774833,  0.990106890301797,  0.990697665394304,  0.991270229274840,  0.991824585069976,  0.992360735809947,  0.992878684428273,  0.993378433761397,
    0.993859986548364,  0.994323345430516,  0.994768512951221,  0.995195491555617,  0.995604283590382,  0.995994891303524,  0.996367316844183,  0.996721562262462,
    0.997057629509263,  0.997375520436144,  0.997675236795187,  0.997956780238879,  0.998220152320008,  0.998465354491564,  0.998692388106653,  0.998901254418423,
    0.999091954579993,  0.999264489644396,  0.999418860564520,  0.999555068193067,  0.999673113282508,  0.999772996485055,  0.999854718352623,  0.999918279336813,
    0.999963679788891,  0.999990919959769,  1,  0.999990919959769,  0.999963679788891,  0.999918279336813,  0.999854718352623,  0.999772996485055,  0.999673113282508,
    0.999555068193067,  0.999418860564520,  0.999264489644396,  0.999091954579993,  0.998901254418423,  0.998692388106653,  0.998465354491564,  0.998220152320008,
    0.997956780238879,  0.997675236795187,  0.997375520436144,  0.997057629509263,  0.996721562262462,  0.996367316844183,  0.995994891303524,  0.995604283590382,
    0.995195491555617,  0.994768512951221,  0.994323345430516,  0.993859986548364,  0.993378433761397,  0.992878684428273,  0.992360735809947,  0.991824585069976,
    0.991270229274840,  0.990697665394304,  0.990106890301797,  0.989497900774833,  0.988870693495458,  0.988225265050742,  0.987561611933306,  0.986879730541886,
    0.986179617181949,  0.985461268066350,  0.984724679316042,  0.983969846960834,  0.983196766940212,  0.982405435104210,  0.981595847214352,  0.980767998944654,
    0.979921885882700,  0.979057503530790,  0.978174847307163,  0.977273912547313,  0.976354694505376,  0.975417188355627,  0.974461389194057,  0.973487292040064,
    0.972494891838247,  0.971484183460316,  0.970455161707115,  0.969407821310782,  0.968342156937030,  0.967258163187578,  0.966155834602719,  0.965035165664051,
    0.963896150797367,  0.962738784375712,  0.961563060722630,  0.960368974115589,  0.959156518789610,  0.957925688941105,  0.956676478731935,  0.955408882293693,
    0.954122893732236,  0.952818507132468,  0.951495716563390,  0.950154516083430,  0.948794899746072,  0.947416861605786,  0.946020395724292,  0.944605496177160,
    0.943172157060767,  0.941720372499633,  0.940250136654151,  0.938761443728727,  0.937254287980364,  0.935728663727692,  0.934184565360483,  0.932621987349668,
    0.931040924257880,  0.929441370750553,  0.927823321607603,  0.926186771735715,  0.924531716181277,  0.922858150143981,  0.921166068991134,  0.919455468272707,
    0.917726343737163,  0.915978691348104,  0.914212507301773,  0.912427788045454,  0.910624530296824,  0.908802731064290,  0.906962387668373,  0.905103497764185,
    0.903226059365053,  0.901330070867355,  0.899415531076618,  0.897482439234952,  0.895530795049877,  0.893560598724629,  0.891571850989997,  0.889564553137791,
    0.887538707056014,  0.885494315265815,  0.883431380960336,  0.881349908045532,  0.879249901183061,  0.877131365835374,  0.874994308313094,  0.872838735824803,
    0.870664656529384,  0.868472079591014,  0.866261015236983,  0.864031474818446,  0.861783470874297,  0.859517017198296,  0.857232128909640,  0.854928822527143,
    0.852607116047223,  0.850267029025881,  0.847908582664905,  0.845531799902485,  0.843136705508510,  0.840723326184753,  0.838291690670240,  0.835841829852038,
    0.833373776881781,  0.830887567298213,  0.828383239156077,  0.825860833161682,  0.823320392815505,  0.820761964562199,  0.818185597948391,  0.815591345788703,
    0.812979264340408,  0.810349413487201,  0.807701856932556,  0.805036662403180,  0.802353901863104,  0.799653651738972,  0.796935993157121,  0.794201012193083,
    0.791448800134157,  0.788679453755749,  0.785893075612205,  0.783089774342900,  0.780269664994384,  0.777432869359426,  0.774579516333846,  0.771709742292047,
    0.768823691482235,  0.765921516442330,  0.763003378437633,  0.760069447921373,  0.757119905019274,  0.754154940039375,  0.751174754008356,  0.748179559235677,
    0.745169579906912,  0.742145052707687,  0.739106227479687,  0.736053367910264,  0.732986752257195,  0.729906674110221,  0.726813443191006,  0.723707386193228,
    0.720588847664505,  0.717458190931938,  0.714315799073024,  0.711162075933727,  0.707997447195494,  0.704822361492980,  0.701637291584216,  0.698442735574913,
    0.695239218198521,  0.692027292153560,  0.688807539499643,  0.685580573113424,  0.682347038205542,  0.679107613899383,  0.675863014872208,  0.672613993058874,
    0.669361339417963,  0.666105885759690,  0.662848506634439,  0.659590121280139,  0.656331695626009,  0.653074244349389,  0.649818832981466,  0.646566580056669,
    0.643318659299357,  0.640076301840126,  0.636840798452607,  0.633613501800058,  0.630395828679254,  0.627189262247287,  0.623995354214764,  0.620815726986607,
    0.617652075729228,  0.614506170340226,  0.611379857293950,  0.608275061333410,  0.605193786975923,  0.602138119796820,  0.599110227452335,  0.596112360399685,
    0.593146852269253,  0.590216119840874,  0.587322662573541,  0.584469061635520,  0.581657978380002,  0.578892152210165,  0.576174397776989,  0.573507601453594,
    0.570894717031323,  0.568338760585497,  0.565842804462912,  0.563409970348817,  0.561043421378528,  0.558746353268044,  0.556521984449196,  0.554373545207901,
    0.552304265839106,  0.550317363848809,  0.548416030251989,  0.546603415035091,  0.544882611872555,  0.543256642208280,  0.541728438834321,  0.540300829119937,
    0.538976518063627,  0.537758071358266,  0.536647898674164,  0.535648237376060,  0.534761136897070,  0.533988443994819,  0.533331789111999,  0.532792574055059,
    0.532371961190630,  0.532070864339646,  0.531889941524406,  0.531829589694499
};


static std::vector<double> coef_b_60_130 =
{
    QString("1.43151420393592e-07").toDouble(), 0, QString("-7.15757101967961e-07").toDouble(), 0, QString("1.43151420393592e-06").toDouble(), 0,
    QString("-1.43151420393592e-06").toDouble(), 0, QString("7.15757101967961e-07").toDouble(), 0, QString("-1.43151420393592e-07").toDouble()
};

static std::vector<double> coef_a_60_130 =
{
    -9.65555675224510, 42.0148649140536, -108.497368511678, 184.135790068398, -214.601866465888,
    173.940029424115, -96.8150214720136, 35.4152659301738, -7.68832217528234, 0.752185040612696
};

static std::vector<double> coef_b_140_200 =
{
    QString("6.75605493210529e-08").toDouble(), 0, QString("-3.37802746605265e-07").toDouble(), 0, QString("6.75605493210529e-07").toDouble(), 0,
    QString("-6.75605493210529e-07").toDouble(), 0, QString("3.37802746605265e-07").toDouble(), 0, QString("-6.75605493210529e-08").toDouble()
};

static std::vector<double> coef_a_140_200 =
{
    -9.54102950930359,	41.1739605730200,	-105.826520308098,	179.391742942936,	-209.559731182444,
    170.844509429156,	-95.9825255936192,	35.5648047449094,	-7.84864746340431,	0.783436514051531
};


static std::vector<double> coef_b_20_80 =
{
    QString("6.73481856464642e-08").toDouble(), 0, QString("-3.36740928232321e-07").toDouble(), 0, QString("6.73481856464642e-07").toDouble(), 0,
    QString("-6.73481856464642e-07").toDouble(), 0, QString("3.36740928232321e-07").toDouble(), 0, QString("-6.73481856464642e-08").toDouble()
};

static std::vector<double> coef_a_20_80 =
{
    -9.74369493628879, 42.7375328949934, -111.122327446432, 189.676149149888, -222.084275400377,
    180.638860681246, -100.785678536059, 36.9153612984597, -8.01536421948142, 0.783436514051529
};

static std::vector<double> coef_b_20_80_4 =
{
    QString("8.98495544083413e-07").toDouble(), 0, QString("-3.59398217633365e-06").toDouble(), 0, QString("5.39097326450048e-06").toDouble(),
    0, QString("-3.59398217633365e-06").toDouble(), 0, QString("8.98495544083413e-07").toDouble()
};

static std::vector<double> coef_a_20_80_4 =
{
    -7.82097673691518, 26.7769272560449, -52.4183396785912, 64.1721864510088,
    -50.3096983872514, 24.6659762851809, -6.91463118855324, 0.848555999266472
};


static std::vector<double> coef_b_20_100 =
{
    QString("2.74676516259387e-07").toDouble(), 0, QString("-1.37338258129693e-06").toDouble(), 0, QString("2.74676516259387e-06").toDouble(), 0,
    QString("-2.74676516259387e-06").toDouble(), 0, QString("1.37338258129693e-06").toDouble(), 0, QString("-2.74676516259387e-07").toDouble()
};


static std::vector<double> coef_a_20_100 =
{
    -9.65942826080248, 42.0064592033137, -108.302515181197, 183.329659743640, -212.898832288214,
    171.773116058154, -95.0790462717348, 34.5532476970420, -7.44483084314317, 0.722170142942136
};


enum SignalPosition {
    SIGNAL_L_BOUND = 0,
    SIGNAL_CENTER = 1,
    SIGNAL_R_BOUND = 2
};

#include <QFile>

template<typename InputIterator>

//using InputIterator = std::vector<double>::iterator;

void fnSelectAxesGroupCenter(InputIterator begin,
                           InputIterator end,
                           std::vector<CarAxisDetect> & axes,
                           std::vector<CrackDetect> & cracks,
                           double max_freq,
                           size_t spectrum_window, size_t spectrum_approx_window,
                           size_t signal_window, size_t signal_approx_window,
                           bool freq_band)
{
    auto size = std::distance(begin, end);

    std::vector<double> data_clean(size);

//    std::vector<CrackDetect> cracks;

//    remove_crack(begin, end, data_clean.begin(), coef_noise_strip, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight, cracks);
    remove_crack(begin, end, data_clean.begin(), coef_noise_strip, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight, cracks/*, 5, 20.0, 0.5*/);
//    remove_crack(begin, end, data_clean.begin(), coef_noise_strip, coef_b_20_80_4, coef_a_20_80_4, coef_noise_strip_hamming_weight, cracks/*, 5, 20.0, 0.5*/);

    auto mean_clean = get_mean(data_clean.begin(), data_clean.end());

    for(auto &value : data_clean) value -= mean_clean;

    std::vector<double> hamming_window;

    create_hamming_window(hamming_window, data_clean.size(), 1.0);

    for(size_t i = 0; i < data_clean.size(); ++i) data_clean[i] *= hamming_window[i];

    std::vector<std::complex<double>> fft_res(size);
    std::vector<double> abs_res(size);

    fftw_plan fft_plan;
    {
        std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
        fft_plan = fftw_plan_dft_r2c_1d(data_clean.size(), data_clean.data(), reinterpret_cast<fftw_complex *>(fft_res.data()), FFTW_ESTIMATE);
    }
    fftw_execute(fft_plan);
    {
        std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
        fftw_destroy_plan(fft_plan);
    }

    for(size_t i = 0; i < fft_res.size(); ++i) {
        abs_res[i] = std::abs(fft_res[i]);
    }


    std::vector<std::tuple<iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>>> freq_pos;

    int count = 0;
    bool stop = false;

    std::vector<double> abs_res_filtered(abs_res.size()/2);

    std::vector<double> tmp(abs_res_filtered.size());

    double pow_thres;

    do {
        std::vector<double> spectrum_filter_coef(spectrum_window, 1.0/spectrum_window);

        simple_filter(abs_res.data(), abs_res.size()/2, abs_res_filtered.data(), spectrum_filter_coef);

        freq_pos.clear();
        mean_linear_approx_new(abs_res_filtered.begin(), abs_res_filtered.end(), tmp.begin(), spectrum_approx_window, -0.4, 0.2, freq_pos, 1.25);

        pow_thres = tmp[std::get<SIGNAL_CENTER>(freq_pos[0])];

        for(auto &tuple : freq_pos) {
            int64_t l = (int64_t)(std::get<SIGNAL_L_BOUND>(tuple)) - spectrum_window/2;
            if(l < 0) l = 0;
            int64_t c = (int64_t)(std::get<SIGNAL_CENTER>(tuple)) - spectrum_window/2;
            int64_t r = (int64_t)(std::get<SIGNAL_R_BOUND>(tuple)) - spectrum_window/2;
            tuple = std::make_tuple(l, c, r);
        }

//        for(size_t i = 0; i < freq_pos.size(); ++i) qDebug() << std::get<SIGNAL_L_BOUND>(freq_pos[i]) << std::get<SIGNAL_CENTER>(freq_pos[i]) << std::get<SIGNAL_R_BOUND>(freq_pos[i]);
//        qDebug() << "=====";

        count = 0;
        for(size_t i = 0; i < freq_pos.size(); ++i) {
            if(5000*std::get<SIGNAL_CENTER>(freq_pos[i])/abs_res.size() < 160) {
                ++count;
            }
        }

        double test = spectrum_window * 1.2;

        spectrum_window = test;

        if(5000.0*spectrum_window/abs_res.size() > 25) {
            stop = true;
            break;
        }

        test = spectrum_approx_window * 1.2;
        spectrum_approx_window = test;

    } while (count > 2);

//     QFile f("test_fft_approx.dat");
//     f.open(QFile::WriteOnly);
//     f.write((char *)tmp.data(), tmp.size()*sizeof(double));
//     f.close();

    int64_t freq_begin;
    int64_t freq_end = max_freq*size/5000;

    if(freq_band) {
        if(stop) {
            freq_begin = 140*size/5000;
        } else {
            if(freq_pos.size() > 1) freq_begin = std::get<SIGNAL_R_BOUND>(freq_pos[1]);
            else freq_begin = std::get<SIGNAL_R_BOUND>(freq_pos[0]);
        }

    } else {
        if(stop) {
            freq_begin = 90*size/5000;
        } else {
            if(freq_pos.size() > 1) freq_begin = std::get<SIGNAL_L_BOUND>(freq_pos[1]);
            else freq_begin = std::get<SIGNAL_R_BOUND>(freq_pos[0]);
        }
    }

    std::vector<std::pair<iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>>> interference_range;

    for(size_t i = 1; i < freq_pos.size(); ++i) {
        if(std::get<SIGNAL_L_BOUND>(freq_pos[i]) < freq_begin) continue;

        if(std::get<SIGNAL_L_BOUND>(freq_pos[i]) > freq_end) break;

        auto pow_current = tmp[std::get<SIGNAL_CENTER>(freq_pos[i])];

        if(pow_current > 1.2*pow_thres/(i+1)) {
            interference_range.push_back(std::make_pair(std::get<SIGNAL_L_BOUND>(freq_pos[i]), std::get<SIGNAL_R_BOUND>(freq_pos[i])));
        }
    }

    for(int64_t i = 0; i < freq_begin; ++i) fft_res[i] = 0;

    for(int64_t i = freq_end; i < static_cast<int64_t>(fft_res.size()/2); ++i) fft_res[i] = 0;

    for(auto range : interference_range) {
        for(int i = range.first; i <= range.second; ++i)
            fft_res[i] = 0;
    }

    for(size_t i = 0, j = fft_res.size()-1; i < fft_res.size()/2; ++i, --j) {
        fft_res[j] = fft_res[i];
    }

    std::vector<double> ifft_res(fft_res.size());

    {
        std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
        fft_plan = fftw_plan_dft_c2r_1d(fft_res.size(), reinterpret_cast<fftw_complex *>(fft_res.data()), ifft_res.data(), FFTW_ESTIMATE);
    }
    fftw_execute(fft_plan);
    {
        std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
        fftw_destroy_plan(fft_plan);
    }

    for(auto &value : ifft_res) value /= ifft_res.size();

    std::vector<double> ifft_res_pow(ifft_res.size());
    std::copy(ifft_res.begin(), ifft_res.end(), ifft_res_pow.begin());

    for(auto &value : ifft_res_pow) value *= value;

    std::vector<double> signal_coef(signal_window, 1.0/signal_window);

    simple_filter(ifft_res_pow.data(), ifft_res_pow.size(), ifft_res_pow.data(), signal_coef);

    std::vector<std::tuple<iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>>> axes_pos;

    mean_linear_approx_new(ifft_res_pow.begin(), ifft_res_pow.end(), data_clean.begin(), signal_approx_window, -0.7, 0.3, axes_pos);

    auto ifft_res_pow_mean = get_mean(ifft_res_pow.begin(), ifft_res_pow.end());

    for(auto &value : data_clean) value /= ifft_res_pow_mean;

//     QFile f("test_ifft_pow_approx2.dat");
//     f.open(QFile::WriteOnly);
//     f.write((char *)data_clean.data(), data_clean.size()*sizeof(double));
//     f.close();

    auto axes_pos_approx = axes_pos;

    for(auto &tuple : axes_pos) {
        int64_t l = std::get<SIGNAL_L_BOUND>(tuple) - signal_window/2;
        if(l < 0) l = 0;
        int64_t c = std::get<SIGNAL_CENTER>(tuple) - signal_window/2;
        int64_t r = std::get<SIGNAL_R_BOUND>(tuple) - signal_window/2;
        tuple = std::make_tuple(l, c, r);
    }

    // for(size_t i = 0; i < axes_pos.size(); ++i) qDebug() << std::get<SIGNAL_L_BOUND>(axes_pos[i]) << std::get<SIGNAL_CENTER>(axes_pos[i]) << std::get<SIGNAL_R_BOUND>(axes_pos[i]);
    // qDebug() << "===";
    // for(size_t i = 0; i < axes_pos_approx.size(); ++i) qDebug() << std::get<SIGNAL_L_BOUND>(axes_pos_approx[i]) << std::get<SIGNAL_CENTER>(axes_pos_approx[i]) << std::get<SIGNAL_R_BOUND>(axes_pos_approx[i]);

    auto res_stdev = get_stdev(ifft_res.begin(), ifft_res.end());

    for(size_t i = 0; i < axes_pos.size(); ++i) {
        CarAxisDetect d;
        d.coord_ax_signal = std::get<SIGNAL_CENTER>(axes_pos[i]);

        auto left_bound = std::distance(ifft_res.begin(), ifft_res.begin() + std::get<SIGNAL_CENTER>(axes_pos[i]) - 100);
        if(left_bound < 0) left_bound = 0;

        auto right_bound = std::distance(ifft_res.begin(), ifft_res.begin() + std::get<SIGNAL_CENTER>(axes_pos[i]) + 100);
        auto max_dist = std::distance(ifft_res.begin(), ifft_res.end());
        if(right_bound > max_dist) right_bound = max_dist;

        d.pow_ax_signal = get_stdev(ifft_res.begin()+left_bound, ifft_res.begin()+right_bound) / res_stdev;

        d.pow_ax_signal_approx = *(data_clean.begin()+std::get<SIGNAL_CENTER>(axes_pos_approx[i]));

        axes.push_back(d);
    }

//    for(auto axis : axes) qDebug() << axis.coord_ax_signal << axis.pow_ax_signal << axis.pow_ax_signal_approx;
}

#endif // SENSOR_DATA_PROCESSOR_H
