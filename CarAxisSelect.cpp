#include <QCoreApplication>
#include <QFile>
#include <iostream>
#include <thread>
#include <QTime>
#include "CarAxis.h"
#include "sensor_data_processor.h"

#include <QDateTime>


void fnCarAxisSelect(std::vector<CarAxisDetect> CarAxis, std::vector<CrackDetect> Crack, size_t LenS, double th0, double th1, int thLen1, int thLen, std::vector<CarAxisDetect> * GroupAx, int & Is_carInterf)
{
    bool car_interf_left = false;
    bool car_interf_right = false;
    Is_carInterf = 0;
    for(std::vector<CarAxisDetect>::iterator it=CarAxis.begin(); it!=CarAxis.end(); ++it){
        if(it->coord_ax_signal < 3000){
            car_interf_left = true;
        }
        if(it->coord_ax_signal > LenS - 3000){
            car_interf_right = true;
        }
    }
    if((car_interf_left)&&(car_interf_right)){
        Is_carInterf = 2;
        return;
    }
    else if((car_interf_left)&&(!car_interf_right)){
        Is_carInterf = 1;
        return;
    }
    else if((!car_interf_left)&&(car_interf_right)){
        Is_carInterf = -1;
        return;
    }
    else{}

    int i=0,j=0;
    bool fl_crack=false, fl_pair_crack;
    if((CarAxis.size()>0)&&(Crack.size())>0){
        int dist_pair_ax=(CarAxis.begin()+1)->coord_ax_signal - CarAxis.begin()->coord_ax_signal;
        int dist_pair_cr=(Crack.begin()+1)->coord_crack_signal - Crack.begin()->coord_crack_signal;
        if((CarAxis.begin()->coord_ax_signal>Crack.begin()->coord_crack_signal)&&((abs(dist_pair_ax-dist_pair_cr))<120)){
            fl_pair_crack=true;
        }
    }
    j=0;
    for(std::vector<CarAxisDetect>::iterator it_ax=CarAxis.begin(); it_ax!=CarAxis.end(); ++it_ax){
        i=0;
        for(std::vector<CrackDetect>::iterator it_cr=Crack.begin(); it_cr!=Crack.end(); ++it_cr){
            int dist_ax_cr = abs(it_ax->coord_ax_signal - it_cr->coord_crack_signal);
            if(dist_ax_cr<100){
                if((i==0)&&(j==0)){
//это помеха от первой оси на трещине облегчаем требования по обнаружению помехи
                    if(((it_ax->pow_ax_signal<th0)&&(it_cr->pow_crack_signal<0.4))||(((fl_pair_crack)||(dist_ax_cr<10))&&(it_cr->pow_crack_signal<0.4))||(((fl_pair_crack)||(dist_ax_cr<25))&&((it_cr->pow_crack_signal<0.4)||(it_ax->pow_ax_signal<3)))||(it_cr->pow_crack_signal<0.2)){
                        it_ax->coord_ax_signal = -1;
                        fl_crack = true;
                        break;
                    }
                }
                else if(j==0) {
//помеха от первой оси на трещине пропущена, это помеха от второй оси на трещине когда первая ось проходит через шум.полосу.
//ужесточаем требования по обнаружению помехи (можем подавить сигнал оси)
                    if(dist_ax_cr<50){
                        if(((it_ax->pow_ax_signal<1.5)&&(it_cr->pow_crack_signal<0.25))||(it_cr->pow_crack_signal<0.15)){
                            it_ax->coord_ax_signal = -1;
                            break;
                        }
                    }
                }
                else if(j==1){
                    if(fl_crack){
//помеха от первой оси на трещине обнаружена,это помеха от второй оси на трещине когда первая ось проходит через шум.полосу
//ужесточаем требования по обнаружению помехи (можем подавить сигнал оси)
                         if(dist_ax_cr<50){
                             if(((it_ax->pow_ax_signal<1.5)&&(it_cr->pow_crack_signal<0.25))||(it_cr->pow_crack_signal<0.15)){
                                 it_ax->coord_ax_signal = -1;
                                 break;
                             }
                         }
                         else if(it_cr->pow_crack_signal<0.1){
                             it_ax->coord_ax_signal = -1;
                             break;
                         }
                         else{}
                    }
                    else{
//помеха от первой оси на трещине подавлена (она не обнаруживается), это сигнал от второй оси на шум.полосе, облегчаем требования по обнаружению помехи
                        if(dist_ax_cr<50){
                            if(((it_ax->pow_ax_signal<th0)&&(it_cr->pow_crack_signal<0.4))||(it_cr->pow_crack_signal<0.2)){
                                it_ax->coord_ax_signal = -1;
                                fl_crack = true;
                                break;
                            }
                        }
                    }
                }
                else if (j>1){
//облегчаем требования по обнаружению помехи
                    if(dist_ax_cr<50){
                        if(((it_ax->pow_ax_signal<th0)&&(it_cr->pow_crack_signal<0.4))||(it_cr->pow_crack_signal<0.2)){
                            it_ax->coord_ax_signal = -1;
                            break;
                        }
                    }
                    else if(it_cr->pow_crack_signal<0.1){
                        it_ax->coord_ax_signal = -1;
                        break;
                    }
                    else{}
                }
                else{}
            }
            i++;
        }
        j++;
    }

//объединение осей в группы
//    std::vector<CarAxisDetect> GroupAx[5];
//первая группа (обычно одна ось)
    bool go_to_next_group = false;
    bool last_ax_wr = false;
    std::vector<CarAxisDetect>::iterator it_ax_j=CarAxis.begin(), it_ax_i=CarAxis.begin();
    while(it_ax_j!=CarAxis.end()){
        if((it_ax_j->pow_ax_signal>th1)&&(it_ax_j->pow_ax_signal_approx>3)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[0].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen1){
//переходим к следующей группе осей
//                        if(dist<2500){
                        if(dist<2000){
//если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            GroupAx[0].clear();
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<3)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[0].push_back(*it_ax_i);
                        if(q==2){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 2){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей в первой группе
    int lenGr1=GroupAx[0].size();
    if (lenGr1>1){
        if(lenGr1==2){
            if(GroupAx[0][1].pow_ax_signal > GroupAx[0][0].pow_ax_signal){
                GroupAx[0][0].coord_ax_signal = -1;
            }
            else{
                GroupAx[0][1].coord_ax_signal = -1;
            }
        }
    }


//вторая группа (обычно одна или две оси, редко 3)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
           break;
        }
        else{
           go_to_next_group=false;
        }
        if((it_ax_j->pow_ax_signal>th1)&&(it_ax_j->pow_ax_signal_approx>3)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[1].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen){
//переходим к следующей группе осей
                        if(dist<2500){
//если расстояние допустимое иначе заканчиваем работу
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<3)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[1].push_back(*it_ax_i);
                        if(q==4){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 4){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                        go_to_next_group=false;
                }
                break;
            }
            else{
                go_to_next_group=false;
                it_ax_j=CarAxis.end();
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей во второй группе
    int lenGr2=GroupAx[1].size();
    if(lenGr2>1){
        if(lenGr2==2){
            if((GroupAx[1][1].pow_ax_signal/GroupAx[1][0].pow_ax_signal > 2.5)||(GroupAx[1][1].pow_ax_signal_approx/GroupAx[1][0].pow_ax_signal_approx > 3)){
                GroupAx[1][0].coord_ax_signal = -1;
            }
            else if((GroupAx[1][0].pow_ax_signal/GroupAx[1][1].pow_ax_signal > 2.5)||(GroupAx[1][0].pow_ax_signal_approx/GroupAx[1][1].pow_ax_signal_approx > 3)){
                GroupAx[1][1].coord_ax_signal = -1;
            }
            else{}
        }
        else if(lenGr2==3){
            if ((GroupAx[1][1].pow_ax_signal/GroupAx[1][0].pow_ax_signal > 2)||(GroupAx[1][1].pow_ax_signal_approx/GroupAx[1][0].pow_ax_signal_approx > 3)){
                GroupAx[1][0].coord_ax_signal = -1;
            }
            if ((GroupAx[1][1].pow_ax_signal/GroupAx[1][2].pow_ax_signal > 2)||(GroupAx[1][1].pow_ax_signal_approx/GroupAx[1][2].pow_ax_signal_approx > 3)){
                GroupAx[1][2].coord_ax_signal = -1;
            }
        }
        else if (lenGr2==4){
            double powMin=1e6;
            int i=0,imin;
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[1].begin();it_ax!=GroupAx[1].end(); ++it_ax){
                if(it_ax->pow_ax_signal < powMin){
                    powMin = it_ax->pow_ax_signal;
                    imin=i;
                }
                i++;
            }
            GroupAx[1][imin].coord_ax_signal = -1;
            int q=0,p=0, coord[3];
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[1].begin();it_ax!=GroupAx[1].end(); ++it_ax){
                if(it_ax->coord_ax_signal!=-1){
                    coord[p]=q;
                    p++;
                }
                q++;
            }
            if ((GroupAx[1][coord[1]].pow_ax_signal/GroupAx[1][coord[0]].pow_ax_signal > 2)||(GroupAx[1][coord[1]].pow_ax_signal_approx/GroupAx[1][coord[0]].pow_ax_signal_approx > 3)){
                GroupAx[1][coord[0]].coord_ax_signal = -1;
            }
            if ((GroupAx[1][coord[1]].pow_ax_signal/GroupAx[1][coord[2]].pow_ax_signal > 2)||(GroupAx[1][coord[1]].pow_ax_signal_approx/GroupAx[1][coord[2]].pow_ax_signal_approx > 3)){
                GroupAx[1][coord[2]].coord_ax_signal = -1;
            }
        }
        else{}
    }

//третья группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        if((it_ax_j->pow_ax_signal>th1)&&(it_ax_j->pow_ax_signal_approx>3)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[2].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen){
//переходим к следующей группе осей
                        if(dist<2500){
//если расстояние допустимое иначе заканчиваем работу
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<3)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[2].push_back(*it_ax_i);
                        if(q==4){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 4){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            else{
                go_to_next_group=false;
                it_ax_j=CarAxis.end();
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей в третьей группе
    int lenGr3=GroupAx[2].size();
    if(lenGr3>1){
        if(lenGr3==2){
            if((GroupAx[2][1].pow_ax_signal/GroupAx[2][0].pow_ax_signal > 2.5)||(GroupAx[2][1].pow_ax_signal_approx/GroupAx[2][0].pow_ax_signal_approx > 3)){
                GroupAx[2][0].coord_ax_signal = -1;
            }
            else if((GroupAx[2][0].pow_ax_signal/GroupAx[2][1].pow_ax_signal > 2.5)||(GroupAx[2][0].pow_ax_signal_approx/GroupAx[2][1].pow_ax_signal_approx > 3)){
                GroupAx[2][1].coord_ax_signal = -1;
            }
            else{}
        }
        else if(lenGr3==3){
            if ((GroupAx[2][1].pow_ax_signal/GroupAx[2][0].pow_ax_signal > 2)||(GroupAx[2][1].pow_ax_signal_approx/GroupAx[2][0].pow_ax_signal_approx > 3)){
                GroupAx[2][0].coord_ax_signal = -1;
            }
            if ((GroupAx[2][1].pow_ax_signal/GroupAx[2][2].pow_ax_signal > 2)||(GroupAx[2][2].pow_ax_signal_approx/GroupAx[2][2].pow_ax_signal_approx > 3)){
                GroupAx[2][2].coord_ax_signal = -1;
            }
        }
        else if (lenGr3==4){
            double powMin=1e6;
            int i=0,imin;
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[2].begin();it_ax!=GroupAx[2].end(); ++it_ax){
                if(it_ax->pow_ax_signal < powMin){
                    powMin = it_ax->pow_ax_signal;
                    imin=i;
                }
                i++;
            }
            GroupAx[2][imin].coord_ax_signal = -1;
            int q=0,p=0,coord[3];
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[2].begin();it_ax!=GroupAx[2].end(); ++it_ax){
                if(it_ax->coord_ax_signal!=-1){
                    coord[p]=q;
                    p++;
                }
                q++;
            }
            if((GroupAx[2][coord[1]].pow_ax_signal/GroupAx[2][coord[0]].pow_ax_signal > 2)||(GroupAx[2][coord[1]].pow_ax_signal_approx/GroupAx[2][coord[0]].pow_ax_signal_approx > 3)){
                GroupAx[2][coord[0]].coord_ax_signal = -1;
            }
            if ((GroupAx[2][coord[1]].pow_ax_signal/GroupAx[2][coord[2]].pow_ax_signal > 2)||(GroupAx[2][coord[1]].pow_ax_signal_approx/GroupAx[2][coord[2]].pow_ax_signal_approx > 3)){
                GroupAx[2][coord[2]].coord_ax_signal = -1;
            }
        }
        else{}
    }

//четвёртая группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        if((it_ax_j->pow_ax_signal>th1)&&(it_ax_j->pow_ax_signal_approx>3)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[3].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen){
//переходим к следующей группе осей
                        if(dist<2500){
//если расстояние допустимое иначе заканчиваем работу
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<3)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[3].push_back(*it_ax_i);
                        if(q==4){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 4){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            else{
                go_to_next_group=false;
                it_ax_j=CarAxis.end();
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей в четвёртой группе
    int lenGr4=GroupAx[3].size();
    if(lenGr4>1){
        if(lenGr4==2){
            if((GroupAx[3][1].pow_ax_signal/GroupAx[3][0].pow_ax_signal > 2.5)||(GroupAx[3][1].pow_ax_signal_approx/GroupAx[3][0].pow_ax_signal_approx > 3)){
                GroupAx[3][0].coord_ax_signal = -1;
            }
            else if((GroupAx[3][0].pow_ax_signal/GroupAx[3][1].pow_ax_signal > 2.5)||(GroupAx[3][0].pow_ax_signal_approx/GroupAx[3][1].pow_ax_signal_approx > 3)){
                GroupAx[3][1].coord_ax_signal = -1;
            }
            else{}
        }
        else if(lenGr4==3){
            if ((GroupAx[3][1].pow_ax_signal/GroupAx[3][0].pow_ax_signal > 2)||(GroupAx[3][1].pow_ax_signal_approx/GroupAx[3][0].pow_ax_signal_approx > 3)){
                GroupAx[3][0].coord_ax_signal = -1;
            }
            if ((GroupAx[3][1].pow_ax_signal/GroupAx[3][2].pow_ax_signal > 2)||(GroupAx[3][2].pow_ax_signal_approx/GroupAx[3][2].pow_ax_signal_approx > 3)){
                GroupAx[3][2].coord_ax_signal = -1;
            }
        }
        else if (lenGr4==4){
            double powMin=1e6;
            int i=0,imin;
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[3].begin();it_ax!=GroupAx[3].end(); ++it_ax){
                if(it_ax->pow_ax_signal < powMin){
                    powMin = it_ax->pow_ax_signal;
                    imin=i;
                }
                i++;
            }
            GroupAx[3][imin].coord_ax_signal = -1;
            int q=0,p=0,coord[3];
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[3].begin();it_ax!=GroupAx[3].end(); ++it_ax){
                if(it_ax->coord_ax_signal!=-1){
                    coord[p]=q;
                    p++;
                }
                q++;
            }
            if((GroupAx[3][coord[1]].pow_ax_signal/GroupAx[3][coord[0]].pow_ax_signal > 2)||(GroupAx[3][coord[1]].pow_ax_signal_approx/GroupAx[3][coord[0]].pow_ax_signal_approx > 3)){
                GroupAx[3][coord[0]].coord_ax_signal = -1;
            }
            if ((GroupAx[3][coord[1]].pow_ax_signal/GroupAx[3][coord[2]].pow_ax_signal > 2)||(GroupAx[3][coord[1]].pow_ax_signal_approx/GroupAx[3][coord[2]].pow_ax_signal_approx > 3)){
                GroupAx[3][coord[2]].coord_ax_signal = -1;
            }
        }
        else{}
    }

//пятая группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        if((it_ax_j->pow_ax_signal>th1)&&(it_ax_j->pow_ax_signal_approx>3)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[4].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen){
//переходим к следующей группе осей
                        if(dist<2500){
//если расстояние допустимое иначе заканчиваем работу
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<3)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[4].push_back(*it_ax_i);
                        if(q==4){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 4){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            else{
                go_to_next_group=false;
                it_ax_j=CarAxis.end();
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей в пятой группе
    int lenGr5=GroupAx[4].size();
    if(lenGr5>1){
        if(lenGr5==2){
            if((GroupAx[4][1].pow_ax_signal/GroupAx[4][0].pow_ax_signal > 2.5)||(GroupAx[4][1].pow_ax_signal_approx/GroupAx[4][0].pow_ax_signal_approx > 3)){
                GroupAx[4][0].coord_ax_signal = -1;
            }
            else if((GroupAx[4][0].pow_ax_signal/GroupAx[4][1].pow_ax_signal > 2.5)||(GroupAx[4][0].pow_ax_signal_approx/GroupAx[4][1].pow_ax_signal_approx > 3)){
                GroupAx[4][1].coord_ax_signal = -1;
            }
            else{}
        }
        else if(lenGr5==3){
            if ((GroupAx[4][1].pow_ax_signal/GroupAx[4][0].pow_ax_signal > 2)||(GroupAx[4][1].pow_ax_signal_approx/GroupAx[4][0].pow_ax_signal_approx > 3)){
                GroupAx[4][0].coord_ax_signal = -1;
            }
            if ((GroupAx[4][1].pow_ax_signal/GroupAx[4][2].pow_ax_signal > 2)||(GroupAx[4][2].pow_ax_signal_approx/GroupAx[4][2].pow_ax_signal_approx > 3)){
                GroupAx[4][2].coord_ax_signal = -1;
            }
        }
        else if (lenGr5==4){
            double powMin=1e6;
            int i=0,imin;
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[4].begin();it_ax!=GroupAx[4].end(); ++it_ax){
                if(it_ax->pow_ax_signal < powMin){
                    powMin = it_ax->pow_ax_signal;
                    imin=i;
                }
                i++;
            }
            GroupAx[4][imin].coord_ax_signal = -1;
            int q=0,p=0,coord[3];
            for(std::vector<CarAxisDetect>::iterator it_ax=GroupAx[4].begin();it_ax!=GroupAx[4].end(); ++it_ax){
                if(it_ax->coord_ax_signal!=-1){
                    coord[p]=q;
                    p++;
                }
                q++;
            }
            if((GroupAx[4][coord[1]].pow_ax_signal/GroupAx[4][coord[0]].pow_ax_signal > 2)||(GroupAx[4][coord[1]].pow_ax_signal_approx/GroupAx[4][coord[0]].pow_ax_signal_approx > 3)){
                GroupAx[4][coord[0]].coord_ax_signal = -1;
            }
            if ((GroupAx[4][coord[1]].pow_ax_signal/GroupAx[4][coord[2]].pow_ax_signal > 2)||(GroupAx[4][coord[1]].pow_ax_signal_approx/GroupAx[4][coord[2]].pow_ax_signal_approx > 3)){
                GroupAx[4][coord[2]].coord_ax_signal = -1;
            }
        }
        else{}
    }
}

void fnCarAxisSelectSmall(std::vector<CarAxisDetect> CarAxis, std::vector<CrackDetect> Crack, size_t LenS, double th0, double th1, int thLen1, int thLen, std::vector<CarAxisDetect> * GroupAx,int & Is_carInterf)
{
    bool car_interf_left = false;
    bool car_interf_right = false;
    Is_carInterf = 0;
    for(std::vector<CarAxisDetect>::iterator it=CarAxis.begin(); it!=CarAxis.end(); ++it){
        if(it->coord_ax_signal < 3000){
            car_interf_left = true;
        }
        if(it->coord_ax_signal > LenS - 3000){
            car_interf_right = true;
        }
    }
    if((car_interf_left)&&(car_interf_right)){
        Is_carInterf = 2;
        return;
    }
    else if((car_interf_left)&&(!car_interf_right)){
        Is_carInterf = 1;
        return;
    }
    else if((!car_interf_left)&&(car_interf_right)){
        Is_carInterf = -1;
        return;
    }
    else{}

    int i=0,j=0;
    bool fl_crack=false, fl_pair_crack;
    if((CarAxis.size()>0)&&(Crack.size())>0){
        int dist_pair_ax=(CarAxis.begin()+1)->coord_ax_signal - CarAxis.begin()->coord_ax_signal;
        int dist_pair_cr=(Crack.begin()+1)->coord_crack_signal - Crack.begin()->coord_crack_signal;
        if((CarAxis.begin()->coord_ax_signal>Crack.begin()->coord_crack_signal)&&((abs(dist_pair_ax-dist_pair_cr))<120)){
            fl_pair_crack=true;
        }
    }
    j=0;
    for(std::vector<CarAxisDetect>::iterator it_ax=CarAxis.begin(); it_ax!=CarAxis.end(); ++it_ax){
        i=0;
        for(std::vector<CrackDetect>::iterator it_cr=Crack.begin(); it_cr!=Crack.end(); ++it_cr){
            int dist_ax_cr = abs(it_ax->coord_ax_signal - it_cr->coord_crack_signal);
            if(dist_ax_cr<100){
                if((i==0)&&(j==0)){
//это помеха от первой оси на трещине облегчаем требования по обнаружению помехи
                    if(((it_ax->pow_ax_signal<3.5)&&(it_cr->pow_crack_signal<0.6))||(((fl_pair_crack)||(dist_ax_cr<25))&&(it_cr->pow_crack_signal<0.4))||(it_cr->pow_crack_signal<0.25)){
                        it_ax->coord_ax_signal = -1;
                        fl_crack = true;
                        break;
                    }
                }
                else if(j==0) {
//помеха от первой оси на трещине пропущена, это помеха от второй оси на трещине когда первая ось проходит через шум.полосу.
//ужесточаем требования по обнаружению помехи (можем подавить сигнал оси)
                    if((dist_ax_cr<50)&&(it_cr->coord_crack_signal>it_ax->coord_ax_signal)){
                        if(((it_ax->pow_ax_signal<2)&&(it_cr->pow_crack_signal<0.4))||(it_cr->pow_crack_signal<0.15)){
                            it_ax->coord_ax_signal = -1;
                            break;
                        }
                    }
                }
                else if(j==1){
                    if(fl_crack){
//помеха от первой оси на трещине обнаружена,это помеха от второй оси на трещине когда первая ось проходит через шум.полосу
//ужесточаем требования по обнаружению помехи (можем подавить сигнал оси)
                         if((dist_ax_cr<50)&&(it_cr->coord_crack_signal>it_ax->coord_ax_signal)){
                             if(((it_ax->pow_ax_signal<2)&&(it_cr->pow_crack_signal<0.4))||(it_cr->pow_crack_signal<0.15)){
                                 it_ax->coord_ax_signal = -1;
                                 break;
                             }
                         }
                         else if(it_cr->pow_crack_signal<0.1){
                             it_ax->coord_ax_signal = -1;
                             break;
                         }
                         else{}
                    }
                    else{
//помеха от первой оси на трещине подавлена (она не обнаруживается), это сигнал от второй оси на шум.полосе, облегчаем требования по обнаружению помехи
                        if(dist_ax_cr<50){
                            if(((it_ax->pow_ax_signal<3.5)&&(it_cr->pow_crack_signal<0.6))||(it_cr->pow_crack_signal<0.25)){
                                it_ax->coord_ax_signal = -1;
                                fl_crack = true;
                                break;
                            }
                        }
                    }
                }
                else if (j>1){
//облегчаем требования по обнаружению помехи
                    if(dist_ax_cr<50){
                        if(((it_ax->pow_ax_signal<3.5)&&(it_cr->pow_crack_signal<0.6))||((dist_ax_cr<25)&&(it_cr->pow_crack_signal<0.25))||(it_cr->pow_crack_signal<0.2)){
                            it_ax->coord_ax_signal = -1;
                            break;
                        }
                    }
                    else if(it_cr->pow_crack_signal<0.1){
                        it_ax->coord_ax_signal = -1;
                        break;
                    }
                    else{}
                }
                else{}
            }
            i++;
        }
        j++;
    }

//объединение осей в группы
//    std::vector<CarAxisDetect> GroupAx[5];
//первая группа (обычно одна ось)
    bool go_to_next_group = false;
    bool last_ax_wr = false;
    std::vector<CarAxisDetect>::iterator it_ax_j=CarAxis.begin(), it_ax_i=CarAxis.begin();
    while(it_ax_j!=CarAxis.end()){
        if((it_ax_j->pow_ax_signal>2)&&(it_ax_j->pow_ax_signal_approx>10)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[0].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen1){
 //переходим к следующей группе осей
//                        if(dist<2500){
                        if(dist<1500){
 //если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            GroupAx[0].clear();
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<th1)||(it_ax_i->pow_ax_signal_approx<10)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[0].push_back(*it_ax_i);
                        if(q==2){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 2){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей в первой группе
    int lenGr1=GroupAx[0].size();
    if (lenGr1>1){
        if(lenGr1==2){
            if(GroupAx[0][1].pow_ax_signal > GroupAx[0][0].pow_ax_signal){
                GroupAx[0][0].coord_ax_signal = -1;
            }
            else{
                GroupAx[0][1].coord_ax_signal = -1;
            }
        }
    }

//вторая группа одна ось
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
           break;
        }
        else{
           go_to_next_group=false;
        }
        if((it_ax_j->pow_ax_signal>2)&&(it_ax_j->pow_ax_signal_approx>10)&&(it_ax_j->coord_ax_signal!=-1)){
            int q=1;
            GroupAx[1].push_back(*it_ax_j);
            std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
            for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
                if(it_ax_i->coord_ax_signal!=-1){
                    int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
                    if(dist > thLen){
//переходим к следующей группе осей
                        if(dist<2000){
//если расстояние допустимое иначе заканчиваем работу
                            go_to_next_group = true;
                            if (it_ax_i == CarAxis.end()){
                                last_ax_wr = true;
                            }
                            break;
                        }
                        else{
                            break;
                        }
                    }
                    else if((dist<150)||(it_ax_i->pow_ax_signal<2)||(it_ax_i->pow_ax_signal_approx<10)){}
                    else{
                        q++;
                        it_ax_imax = it_ax_i;
                        GroupAx[1].push_back(*it_ax_i);
                        if(q==2){
                            go_to_next_group = true;
                            it_ax_j = it_ax_i+1;
                            break;
                        }
                    }
                }
            }
            if(q == 4){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                    go_to_next_group=false;
                }
                break;
            }
            it_ax_j = it_ax_i;
            if(go_to_next_group){
                if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                        go_to_next_group=false;
                }
                break;
            }
            else{
                go_to_next_group=false;
                it_ax_j=CarAxis.end();
            }
        }
        else{
            it_ax_j++;
        }
    }
//дополнительная проверка осей во второй группе
    int lenGr2=GroupAx[1].size();
    if(lenGr2>1){
        if(lenGr2==2){
            if(GroupAx[1][1].pow_ax_signal > GroupAx[1][0].pow_ax_signal){
                GroupAx[1][0].coord_ax_signal = -1;
            }
            else{
                GroupAx[1][1].coord_ax_signal = -1;
            }
        }
    }
}

void fnFillCarAxisVec(std::vector<CarAxisDetect> & CarAxis, int * coord, int num_coord, double * pow_sign, double * pow_sign_apr)
{
    for (int i=0; i<num_coord; i++){
        CarAxisDetect car_ax;
        car_ax.coord_ax_signal=coord[i];
        car_ax.pow_ax_signal=pow_sign[i];
        car_ax.pow_ax_signal_approx=pow_sign_apr[i];
        CarAxis.push_back(car_ax);
    }
}

void fnFillCrackVec(std::vector<CrackDetect> & Crack, int * coord, int num_coord, double * pow_sign, double * pow_sign_apr){
    for (int i=0; i<num_coord; i++){
        CrackDetect crack;
        crack.coord_crack_signal=coord[i];
        crack.pow_crack_signal=pow_sign[i];
        crack.pow_crack_signal_approx=pow_sign_apr[i];
        Crack.push_back(crack);
    }
}


void fnSelectAxisGroupRes(std::vector<int32_t> data, double FreqMax, int LenSpWnd, int LenWndS, int LenWndPow, int LenWndApprox, int IsTruc, bool & Is_Axis_Found, std::vector<CarAxisDetect> * GroupAx, int & Shift)
{
    std::vector<CarAxisDetect> GroupAxFreqBnd_0[5];
    std::vector<CarAxisDetect> GroupAxFreqBnd_1[5];

    Is_Axis_Found = true;

    std::vector<double> test;
    for(std::vector<int32_t>::iterator it = data.begin(); it!=data.end(); ++it){
        test.push_back(1.0 * (*it));
    }
    size_t size = test.size();

    std::vector<CarAxisDetect> AxisFreqBnd_0;
    std::vector<CarAxisDetect> AxisFreqBnd_1;
    std::vector<CrackDetect> Crack1, Crack2;

    fnSelectAxesGroupCenter(test.begin(), test.end(), AxisFreqBnd_0, Crack1, FreqMax, LenSpWnd, LenWndS, LenWndPow, LenWndApprox, 0);

    if(IsTruc != -1){
        fnCarAxisSelect(AxisFreqBnd_0, Crack1, size, 2.5, 1.8, 300, 600, GroupAxFreqBnd_0, Shift);
    }
    else{
        fnCarAxisSelectSmall(AxisFreqBnd_0, Crack1, size, 3.5, 2.0, 300, 600, GroupAxFreqBnd_0, Shift);
    }


    fnSelectAxesGroupCenter(test.begin(), test.end(), AxisFreqBnd_1, Crack2, FreqMax, LenSpWnd, LenWndS, LenWndPow, LenWndApprox, 1);

    if(IsTruc != -1){
        fnCarAxisSelect(AxisFreqBnd_1, Crack2, size, 2.5, 1.8, 300, 600, GroupAxFreqBnd_1, Shift);
    }
    else{
        fnCarAxisSelectSmall(AxisFreqBnd_1, Crack2, size, 3.5, 2.0, 300, 600, GroupAxFreqBnd_1, Shift);
    }

//сливаем все группы осей в один массив в 0 част. диапазоне
    std::vector<CarAxisDetect> GroupAx_Sum;
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_0[0].begin(); it_ax!=GroupAxFreqBnd_0[0].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_0[1].begin(); it_ax!=GroupAxFreqBnd_0[1].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_0[2].begin(); it_ax!=GroupAxFreqBnd_0[2].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_0[3].begin(); it_ax!=GroupAxFreqBnd_0[3].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_0[4].begin(); it_ax!=GroupAxFreqBnd_0[4].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }

//сливаем все группы осей в один массив в 1 част. диапазоне
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_1[0].begin(); it_ax!=GroupAxFreqBnd_1[0].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_1[1].begin(); it_ax!=GroupAxFreqBnd_1[1].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_1[2].begin(); it_ax!=GroupAxFreqBnd_1[2].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_1[3].begin(); it_ax!=GroupAxFreqBnd_1[3].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAxFreqBnd_1[4].begin(); it_ax!=GroupAxFreqBnd_1[4].end(); ++it_ax){
        if(it_ax->coord_ax_signal != -1){
            GroupAx_Sum.push_back(*it_ax);
        }
    }

    if((GroupAx_Sum.size()==0)||(GroupAx_Sum.size()==1)){
        Is_Axis_Found = false;
        return;
    }


    std::sort(GroupAx_Sum.begin(),GroupAx_Sum.end(),[](CarAxisDetect elem1, CarAxisDetect elem2) {
        return elem1.coord_ax_signal < elem2.coord_ax_signal;
    });
//объединяем близкие по координатам оси
    std::vector<CarAxisDetect> GroupAx_Res;
    std::vector<CarAxisDetect>::iterator it_ax = GroupAx_Sum.begin();
    int coord_t = it_ax->coord_ax_signal;
    int coord=coord_t;
    double max_pow_signal = it_ax->pow_ax_signal;
    double max_pow_signal_approx = it_ax->pow_ax_signal_approx;
    it_ax++;
    int q=1;
    for(; it_ax!=GroupAx_Sum.end(); ++it_ax){
        if(abs(it_ax->coord_ax_signal-coord_t)<200){
            coord+=it_ax->coord_ax_signal;
            max_pow_signal = std::max(max_pow_signal,it_ax->pow_ax_signal);
            max_pow_signal_approx = std::max(max_pow_signal_approx,it_ax->pow_ax_signal_approx);
            q++;
            coord_t = floor(coord/q);
        }
        else{
            CarAxisDetect ax;
            ax.coord_ax_signal = floor(coord/q);
            ax.pow_ax_signal = max_pow_signal;
            ax.pow_ax_signal_approx = max_pow_signal_approx;
            GroupAx_Res.push_back(ax);
            coord_t = it_ax->coord_ax_signal;
            coord = coord_t;
            max_pow_signal = it_ax->pow_ax_signal;
            max_pow_signal_approx = it_ax->pow_ax_signal_approx;
            q=1;
        }
    }
    CarAxisDetect ax;
    ax.coord_ax_signal = floor(coord/q);
    ax.pow_ax_signal = max_pow_signal;
    ax.pow_ax_signal_approx = max_pow_signal_approx;
    GroupAx_Res.push_back(ax);
//отбрасываем оси с малой мощностью сигнала
    std::vector<CarAxisDetect> CarAxis;
    for(std::vector<CarAxisDetect>::iterator it_ax = GroupAx_Res.begin(); it_ax!=GroupAx_Res.end(); ++it_ax){
        if((it_ax->pow_ax_signal>1.5)&&(it_ax->pow_ax_signal_approx>3)){
            CarAxis.push_back(*it_ax);
        }
    }

//вновь образуем группы
//первая группа (обычно одна ось)
    bool go_to_next_group = false;
    bool last_ax_wr = false;
    std::vector<CarAxisDetect>::iterator it_ax_j=CarAxis.begin(), it_ax_i=CarAxis.begin();
    while(it_ax_j!=CarAxis.end()){
        int q=1;
        GroupAx[0].push_back(*it_ax_j);
        std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
        for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
            int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
            if(dist > 300){
 //переходим к следующей группе осей
                if(dist<2000){
 //если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                    go_to_next_group = true;
                    if (it_ax_i == CarAxis.end()){
                        last_ax_wr = true;
                    }
                    break;
                }
                else{
                    GroupAx[0].clear();
                    break;
                }
            }
            else{
                q++;
                it_ax_imax = it_ax_i;
                if(dist <= 250){
                    GroupAx[0][0].coord_ax_signal = (GroupAx[0][0].coord_ax_signal + it_ax_i->coord_ax_signal) / 2;
                    GroupAx[0][0].pow_ax_signal = std::max(GroupAx[0][0].pow_ax_signal, it_ax_i->pow_ax_signal);
                    GroupAx[0][0].pow_ax_signal_approx = std::max(GroupAx[0][0].pow_ax_signal_approx, it_ax_i->pow_ax_signal_approx);
                }
                else{
                    GroupAx[0].push_back(*it_ax_i);
                }
                if(q == 2){
                    go_to_next_group = true;
                    it_ax_j = it_ax_i+1;
                    break;
                }
            }
        }
        if(q == 2){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        it_ax_j = it_ax_i;
        if(go_to_next_group){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
    }
//вторая группа (обычно одна или две оси, редко 3)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
           break;
        }
        else{
           go_to_next_group=false;
        }
        int q=1;
        GroupAx[1].push_back(*it_ax_j);
        std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
        for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
            int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
            if(dist > 600){
 //переходим к следующей группе осей
                if(dist<2500){
 //если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                    go_to_next_group = true;
                    if (it_ax_i == CarAxis.end()){
                        last_ax_wr = true;
                    }
                    break;
                }
                else{
                    break;
                }
            }
            else{
                q++;
                it_ax_imax = it_ax_i;
                GroupAx[1].push_back(*it_ax_i);
                if(q==4){
                    go_to_next_group = true;
                    it_ax_j = it_ax_i+1;
                    break;
                }
            }
        }
        if(q == 4){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        it_ax_j = it_ax_i;
        if(go_to_next_group){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        else{
            go_to_next_group=false;
            it_ax_j=CarAxis.end();
        }
    }
//третья группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        int q=1;
        GroupAx[2].push_back(*it_ax_j);
        std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
        for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
            int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
            if(dist > 600){
//переходим к следующей группе осей
                if(dist<2500){
//если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                    go_to_next_group = true;
                    if (it_ax_i == CarAxis.end()){
                        last_ax_wr = true;
                    }
                    break;
                }
                else{
                    break;
                }
            }
            else{
                q++;
                it_ax_imax = it_ax_i;
                GroupAx[2].push_back(*it_ax_i);
                if(q==4){
                    go_to_next_group = true;
                    it_ax_j = it_ax_i+1;
                    break;
                }
            }
        }
        if(q == 4){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        it_ax_j = it_ax_i;
        if(go_to_next_group){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        else{
            go_to_next_group=false;
            it_ax_j=CarAxis.end();
        }
    }
//четвёртая группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        int q=1;
        GroupAx[3].push_back(*it_ax_j);
        std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
        for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
            int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
            if(dist > 600){
//переходим к следующей группе осей
                if(dist<2500){
//если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                    go_to_next_group = true;
                    if (it_ax_i == CarAxis.end()){
                       last_ax_wr = true;
                    }
                    break;
                }
                else{
                    break;
                }
            }
            else{
                q++;
                it_ax_imax = it_ax_i;
                GroupAx[3].push_back(*it_ax_i);
                if(q==4){
                    go_to_next_group = true;
                    it_ax_j = it_ax_i+1;
                    break;
                }
            }
        }
        if(q == 4){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        it_ax_j = it_ax_i;
        if(go_to_next_group){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        else{
            go_to_next_group=false;
            it_ax_j=CarAxis.end();
        }
    }

//пятая группа (обычно две или три оси, редко одна)
    while(it_ax_j!=CarAxis.end()){
        if ((it_ax_j == CarAxis.end())&&(!go_to_next_group)){
            break;
        }
        else{
            go_to_next_group=false;
        }
        int q=1;
        GroupAx[4].push_back(*it_ax_j);
        std::vector<CarAxisDetect>::iterator it_ax_imax = it_ax_j;
        for(it_ax_i=it_ax_j+1; it_ax_i!=CarAxis.end(); it_ax_i++){
            int dist = it_ax_i->coord_ax_signal - it_ax_imax->coord_ax_signal;
            if(dist > 600){
//переходим к следующей группе осей
                if(dist<2500){
//если расстояние допустимое иначе это помеха и переходим к следующей "первой оси"
                    go_to_next_group = true;
                    if (it_ax_i == CarAxis.end()){
                       last_ax_wr = true;
                    }
                    break;
                }
                else{
                    break;
                }
            }
            else{
                q++;
                it_ax_imax = it_ax_i;
                GroupAx[4].push_back(*it_ax_i);
                if(q==4){
                    go_to_next_group = true;
                    it_ax_j = it_ax_i+1;
                    break;
                }
            }
        }
        if(q == 4){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        it_ax_j = it_ax_i;
        if(go_to_next_group){
            if((it_ax_j == CarAxis.end())&&(!last_ax_wr)){
                go_to_next_group=false;
            }
            break;
        }
        else{
            go_to_next_group=false;
            it_ax_j=CarAxis.end();
        }
    }


//удаляем лишние группы c 4-й
    if((GroupAx[4].size() != 0)&&(GroupAx[3].size() == 0)){
        GroupAx[4].clear();
        return;
    }
    else{
        if((GroupAx[4].size()==1)&&(GroupAx[3].size()==3)){
            GroupAx[4].clear();
        }
        else if((GroupAx[4].size()==1)&&(GroupAx[3].size()==2)){
            if(abs(GroupAx[4][0].coord_ax_signal-GroupAx[3][1].coord_ax_signal)<1200){
                GroupAx[3].push_back(GroupAx[4][0]);
            }
            GroupAx[4].clear();
        }
        else if((GroupAx[4].size()==1)&&(GroupAx[3].size()==1)){
            if(abs(GroupAx[4][0].coord_ax_signal-GroupAx[3][0].coord_ax_signal)<1200){
                GroupAx[3].push_back(GroupAx[4][0]);
            }
            else{
                if(GroupAx[4][0].pow_ax_signal>GroupAx[3][0].pow_ax_signal){
                    GroupAx[3][0]=GroupAx[4][0];
                }
            }
            GroupAx[4].clear();
        }
        else if((GroupAx[4].size()==2)&&(GroupAx[3].size()==1)){
            if(abs(GroupAx[4][0].coord_ax_signal-GroupAx[3][0].coord_ax_signal)<1200){
                GroupAx[3].push_back(GroupAx[4][0]);
                GroupAx[3].push_back(GroupAx[4][1]);
            }
            else{
                GroupAx[3][0]=GroupAx[4][0];
                GroupAx[3].push_back(GroupAx[4][1]);
            }
            GroupAx[4].clear();
        }
        else if((GroupAx[4].size()==3)&&(GroupAx[3].size()==1)){
            GroupAx[3][0]=GroupAx[4][0];
            GroupAx[3].push_back(GroupAx[4][1]);
            GroupAx[3].push_back(GroupAx[4][2]);
            GroupAx[4].clear();
        }
        else{}
    }


//удаляем лишние группы c 3-й
    if(GroupAx[3].size() == 0){
        return;
    }
    else{
       if((GroupAx[3].size()==1)&&(GroupAx[2].size()==3)){
           GroupAx[3].clear();
       }
       else if((GroupAx[3].size()==1)&&(GroupAx[2].size()==2)){
           if(abs(GroupAx[3][0].coord_ax_signal-GroupAx[2][1].coord_ax_signal)<1200){
               GroupAx[2].push_back(GroupAx[3][0]);
           }
           GroupAx[3].clear();
       }
       else if((GroupAx[3].size()==1)&&(GroupAx[2].size()==1)){
           if(abs(GroupAx[3][0].coord_ax_signal-GroupAx[2][0].coord_ax_signal)<1200){
               GroupAx[2].push_back(GroupAx[3][0]);
           }
           else{
               if(GroupAx[3][0].pow_ax_signal>GroupAx[2][0].pow_ax_signal){
                   GroupAx[2][0]=GroupAx[3][0];
               }
           }
           GroupAx[3].clear();
       }
       else if((GroupAx[3].size()==2)&&(GroupAx[2].size()==1)){
           if(abs(GroupAx[3][0].coord_ax_signal-GroupAx[2][0].coord_ax_signal)<1200){
               GroupAx[2].push_back(GroupAx[3][0]);
               GroupAx[2].push_back(GroupAx[3][1]);
           }
           else{
               GroupAx[2][0]=GroupAx[3][0];
               GroupAx[2].push_back(GroupAx[3][1]);
           }
           GroupAx[3].clear();
       }
       else if((GroupAx[3].size()==3)&&(GroupAx[2].size()==1)){
           GroupAx[2][0]=GroupAx[3][0];
           GroupAx[2].push_back(GroupAx[3][1]);
           GroupAx[2].push_back(GroupAx[3][2]);
           GroupAx[3].clear();
       }
       else{}
    }
}

void fnGetCarPowCenter(const std::vector<int32_t> &data, size_t ProcWnd, size_t LenWndPow, size_t LenWndApprox, size_t FreqWnd, size_t ThLenPulseGroup, bool StrOrLat, std::vector<PulseGroup> & pulse_groups, std::vector<double> & signal_abs_flt, int mean_pow)
{
    using InputIterator = std::vector<double>::iterator;
    using value_type_input = iterator_value_t<InputIterator>;

    std::vector<double> signal;

    size_t size_all = data.size();
    size_t size;

//    for (size_t i=0; i<size_all/10; i+= ProcWnd) {
    for (size_t i = 0; i < size_all - ProcWnd; i+= ProcWnd) {

        std::vector<double> test;
        for(size_t j = i, k=0; k < ProcWnd; ++j, ++k) test.push_back(1.0*data[j]);

        size_t size = test.size();

        std::vector<double> data_clean(size);
        std::vector<CrackDetect> cracks;

        if(!StrOrLat){
            remove_crack(test.begin(), test.end(), data_clean.begin(), coef_noise_strip, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight, cracks, 5, 20.0, 0.5);
        }
        else{
            remove_crack(test.begin(), test.end(), data_clean.begin(), coef_noise_latt, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight_lat, cracks, 5, 20.0, 0.8);
        }
        auto mean_clean = get_mean(data_clean.begin(), data_clean.end());

        for(std::vector<double>::iterator it = data_clean.begin(); it != data_clean.end(); ++it){
            signal.push_back(*it - mean_clean);
        }
    }

    size = signal.size();
    std::vector<double> signal_tmp(size);

    if(StrOrLat){
        passband_filter<value_type_input> f;
        f.set_b(coef_b_60_130);
        f.set_a(coef_a_60_130);
        f.filter(signal.data(), size, signal_tmp.data());
    }
    else{
        passband_filter<value_type_input> f;
        f.set_b(coef_b_140_200);
        f.set_a(coef_a_140_200);
        f.filter(signal.data(), size, signal_tmp.data());
    }

    signal.clear();

    for(std::vector<double>::iterator it=signal_tmp.begin(); it!=signal_tmp.end(); ++it){
        signal.push_back(std::abs(*it));
    }

    std::vector<double> filter_coef(LenWndPow, 1.0/LenWndPow);
    simple_filter(signal.data(), size, signal.data(), filter_coef);

    std::vector<std::tuple<iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>, iterator_difference_t<InputIterator>>> axes_pos;

    if(!StrOrLat){
        mean_linear_approx_new(signal.begin(), signal.end(), signal_abs_flt.begin(), LenWndApprox, -0.5, 0.5, axes_pos, 1.0, mean_pow);
    }
    else{
        mean_linear_approx_new(signal.begin(), signal.end(), signal_abs_flt.begin(), LenWndApprox, -0.5, 0.5, axes_pos, 1.0, mean_pow);
    }

    std::vector<int> CntAxisCoord;
    for(size_t i = 0; i < axes_pos.size(); ++i) {
        CntAxisCoord.push_back(std::get<SIGNAL_CENTER>(axes_pos[i]));
    }


    int q=1;
    std::vector<int>::iterator it=CntAxisCoord.begin();
    PulseGroup pulse_gr;
    Pulses pulse;
    int coord =*it;
    int coord_bg = coord;
    pulse_gr.num_pulse_group = q;
    pulse_gr.freq = 0;
    pulse_gr.freq_stat = 0;
    pulse.coord_pulse = coord;
    pulse.pow_pulse =signal_abs_flt[coord];
    pulse_gr.pulses.push_back(pulse);
    it++;
    for(; it!=CntAxisCoord.end(); ++it){
        if(*it - coord_bg < ThLenPulseGroup){
            coord =*it;
            coord_bg = coord;
            pulse.coord_pulse =coord;
            pulse.pow_pulse=signal_abs_flt[coord];
            pulse_gr.pulses.push_back(pulse);
        }
        else{
            std::sort(pulse_gr.pulses.begin(),pulse_gr.pulses.end(),[](Pulses elem1,Pulses elem2){
                return elem1.pow_pulse > elem2.pow_pulse;
            });
//            pulse_gr.it_pulses_max_freq = pulse_gr.pulses.begin();
            pulse_gr.num_pulses_max_freq_stat = 0;
            pulse_groups.push_back(pulse_gr);
            q++;
            coord = *it;
            coord_bg = coord;
            pulse_gr.pulses.clear();
            pulse_gr.num_pulse_group=q;
            pulse_gr.freq = 0;
            pulse_gr.freq_stat = 0;
            pulse.coord_pulse = coord;
            pulse.pow_pulse = signal_abs_flt[coord];
            pulse_gr.pulses.push_back(pulse);
        }
    }
    std::sort(pulse_gr.pulses.begin(),pulse_gr.pulses.end(),[](Pulses elem1,Pulses elem2){
        return elem1.pow_pulse > elem2.pow_pulse;
    });
    pulse_gr.num_pulses_max_freq_stat = 0;
    pulse_groups.push_back(pulse_gr);

    if(StrOrLat){
        for(std::vector<PulseGroup>::iterator it=pulse_groups.begin(); it!=pulse_groups.end(); ++it){
            PulseGroup pulse_gr = *it;
            double max_freq_stat=0;
            double max_freq=0;
            int num_pulse_max_freq_stat = 0;
            int num_pulse=0;
            for(std::vector<Pulses>::iterator it_p=pulse_gr.pulses.begin(); it_p!=pulse_gr.pulses.end(); ++it_p, num_pulse++){
                std::vector<double> Smp;
                for(int i = it_p->coord_pulse-floor(FreqWnd/2)-LenWndPow/2+1; i<it_p->coord_pulse+floor(FreqWnd/2)-LenWndPow/2+1; i++){
                    Smp.push_back(signal_tmp[i]);
                }

                size_t size = Smp.size();
                std::vector<std::complex<double>> fft_res(size);
                std::vector<double> abs_res(size/2);
                fftw_plan fft_plan;
                {
                    std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
                    fft_plan = fftw_plan_dft_r2c_1d(size, Smp.data(), reinterpret_cast<fftw_complex *>(fft_res.data()), FFTW_ESTIMATE);
                }
                fftw_execute(fft_plan);
                {
                    std::lock_guard<std::mutex> fft_planer_lock(fft_planer_mutex);
                    fftw_destroy_plan(fft_plan);
                }
                for(size_t i = 0; i < fft_res.size()/2; ++i) {
                    abs_res[i] = std::abs(fft_res[i]);
                }

                double max_value_spe=0, freq=0, SumSpe=0;
                std::vector<double> Spe;
                std::vector<double>::iterator it_spe_max;
                int q=0,qmax;
                for(std::vector<double>::iterator it_spe=abs_res.begin(); it_spe!=abs_res.end(); ++it_spe){
                    if (*it_spe > max_value_spe){
                        it_spe_max =it_spe;
                        max_value_spe = *it_spe;
                        qmax=q;
                        freq = q*5000.0/FreqWnd;
                    }
                    SumSpe+=*it_spe;
                    q++;
                }

                double tmp= (*(it_spe_max--)+*(it_spe_max)+*(it_spe_max++))/SumSpe;
                if(tmp > max_freq_stat){
                    max_freq_stat = tmp;
                    max_freq = freq;
                    num_pulse_max_freq_stat = num_pulse;
                }
            }
            it->freq = max_freq;
            it->freq_stat = max_freq_stat;
            it->num_pulses_max_freq_stat = num_pulse_max_freq_stat;
        }
    }
}

union DataWord
{
    struct
    {
        uint32_t channel : 8;
        int32_t value : 24;
    } data;
    uint32_t word;
};

void fnGetCarInfo(QString dataFileName, std::vector<CarInfo> & cars_info, ProgressBarHelper *progressBarHelper)
{
    if(progressBarHelper) emit progressBarHelper->progress(0);

    int Y, M, D, h, m, s, ms;

    // QFile file0("TSt.dat");
    // file0.open(QFile::ReadOnly);
    // QByteArray TSt = file0.readAll();
    // file0.close();
    // const int64_t * TStDat = reinterpret_cast<const int64_t *>(TSt.constData());
    // size_t size_all_tst = TSt.size()/sizeof(int64_t);


    // QFile file("stript.dat");
    // file.open(QFile::ReadOnly);
    // QByteArray dat_stript = file.readAll();
    // file.close();
    // const int32_t *data_stript = reinterpret_cast<const int32_t *>(dat_stript.constData());
    // size_t size_all_stript = dat_stript.size()/sizeof(int32_t);

    // QFile file1("latt.dat");
    // file1.open(QFile::ReadOnly);
    // QByteArray dat_latt = file1.readAll();
    // file1.close();
    // const int32_t *data_latt = reinterpret_cast<const int32_t *>(dat_latt.constData());
    // size_t size_all_latt = dat_latt.size()/sizeof(int32_t);

    QFile dataFile(dataFileName);
    dataFile.open(QFile::ReadOnly);

    auto dataBytes = dataFile.readAll();

    auto data = dataBytes.constData();

    int blockSize = 4096;
    int wordCount = blockSize / sizeof(DataWord);
    int stepSize = blockSize+sizeof(uint64_t);

    std::vector<int32_t> stripData, latData;

    // for(int i = sizeof(uint64_t); i < dataBytes.size(); i += stepSize) {
    //     const DataWord *dataBlock = reinterpret_cast<const DataWord *>(&data[i]);

    //     for(int j = 0; j < wordCount; ++j) {
    //         if(dataBlock[j].data.channel == 0) stripData.push_back(dataBlock[j].data.value);
    //         else if(dataBlock[j].data.channel == 3) latData.push_back(dataBlock[j].data.value);
    //     }
    // }

    #pragma omp parallel
    {
#if defined(_OPENMP)
        std::vector<int32_t> stripDataPriv, latDataPriv;
#endif
        #pragma omp for
        for(int i = sizeof(uint64_t); i < dataBytes.size(); i += stepSize) {
            const DataWord *dataBlock = reinterpret_cast<const DataWord *>(&data[i]);

            for(int j = 0; j < wordCount; ++j) {
                if(dataBlock[j].data.channel == 0) {
#if defined(_OPENMP)
                    stripDataPriv.push_back(dataBlock[j].data.value);
#else
                    stripData.push_back(dataBlock[j].data.value);
#endif
                }
                else if(dataBlock[j].data.channel == 3) {
#if defined(_OPENMP)
                    latDataPriv.push_back(dataBlock[j].data.value);
#else
                    latData.push_back(dataBlock[j].data.value);
#endif
                }
            }
        }
#if defined(_OPENMP)
        #pragma omp for ordered
        for(int i=0; i< omp_get_num_threads(); i++) {
        #pragma omp ordered
            stripData.insert(stripData.end(), stripDataPriv.begin(), stripDataPriv.end());
            latData.insert(latData.end(), latDataPriv.begin(), latDataPriv.end());
        }
#endif
    }

    const uint64_t *timestamp = reinterpret_cast<const uint64_t *>(data);
    QDateTime dt = QDateTime::fromMSecsSinceEpoch(*timestamp);


    //TEST
    // Y = 2023;
    // M = 10;
    // D = 23;
    // h = 9;
    // m = 53;
    // s = 13;
    // ms = 654;

    Y = dt.date().year();
    M = dt.date().month();
    D = dt.date().day();
    h = dt.time().hour();
    m = dt.time().minute();
    s = dt.time().second();
    ms = dt.time().msec();

    // qDebug() << Y << M << D << h << m << s << ms;
    // qDebug() << dt.date().year() << dt.date().month() << dt.date().day() << dt.time().hour() << dt.time().minute() << dt.time().second() << dt.time().msec();

    const int32_t *data_stript = stripData.data();
    size_t size_all_stript = stripData.size();

    const int32_t *data_latt = latData.data();
    size_t size_all_latt = latData.size();

    size_t  size_all = std::min(size_all_stript, size_all_latt);

    size_t size_proc = WIN_SMP_PROCESS; // 5 минут при 5000 Гц
    bool Is_AxisFound;

    if(size_all < size_proc){
        std::vector<int32_t> dat(size_all);
        std::vector<double> signal_abs_flt_str(size_all);
        std::vector<double> signal_abs_flt_lat(size_all);

        std::vector<PulseGroup> pulse_groups_stript;
        std::vector<PulseGroup> pulse_groups_latt;
        std::vector<UnionPulseGroup> pulse_groups_un;

        std::copy(data_stript, data_stript + size_all - 1, dat.data());
        fnGetCarPowCenter(dat, 10000, 200, 100, 1024, 5000, 0, pulse_groups_stript, signal_abs_flt_str, 24743);

        std::copy(data_latt, data_latt + size_all - 1, dat.data());
        fnGetCarPowCenter(dat, 10000, 1000, 500, 1024, 5000, 1, pulse_groups_latt, signal_abs_flt_lat, 84763);

        fnUnionPulsesGroup(pulse_groups_stript, pulse_groups_latt, 0, signal_abs_flt_lat, 5000, pulse_groups_un);

//выделение осей каждого обнаруженного автомобиля
        int num_car=0;
        for(std::vector<UnionPulseGroup>::iterator  it_pulse_groups=pulse_groups_un.begin(); it_pulse_groups!=pulse_groups_un.end(); ++it_pulse_groups){
             std::vector<CarAxisDetect> GroupAx[MAX_NUM_AXIS_GR];
             CarInfo car_info;

             car_info.num_car = num_car;
             car_info.speed = it_pulse_groups->freq * 0.25;
             car_info.num_pulse_group = it_pulse_groups->num_pulse_group;

             int coord_bound_left, coord_bound_right;
             int IsTruc = 0;
             coord_bound_left = it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].coord_pulse - floor( 15.0 / ( it_pulse_groups->freq * 0.25 ) * 5000 ) - 7000;
             coord_bound_left -=500;
             coord_bound_right = it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].coord_pulse + floor( 10.0 /( it_pulse_groups->freq * 0.25 ) * 5000 ) + 5000;
             coord_bound_right -=500;
             if(it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].pow_pulse > 4e5){
                 IsTruc = 1;
                 coord_bound_left -= 5000;
                 coord_bound_right += 5000;
             }
             else if(it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].pow_pulse < 2e5){
                IsTruc = -1;
             }
             else{
                IsTruc = 0;
                coord_bound_left -= 3000;
                coord_bound_right += 3000;
             }

             coord_bound_left = std::max(0, coord_bound_left);
             const int tmp = size_all;
             coord_bound_right = std::min(coord_bound_right, tmp);

             int Shift = 0;

             size_t size_ax_sel = coord_bound_right - coord_bound_left + 1;
             std::vector<int32_t> dat(size_ax_sel);
             std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());
             fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);

             if( Shift != 0){

                 if(Shift == 2){
                     coord_bound_left += 3000;
                     coord_bound_right -= 3000;
                 }
                 else if(Shift == 1){
                     coord_bound_left += 3000;
                 }
                 else if(Shift == -1){
                     coord_bound_right -= 3000;
                 }
                 else{}

                 coord_bound_left = std::max(0, coord_bound_left);
                 const int tmp = size_all;
                 coord_bound_right = std::min(coord_bound_right, tmp);

                 size_ax_sel = coord_bound_right - coord_bound_left + 1;
                 dat.resize(size_ax_sel);
                 std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());

                 for(int j=0; j<MAX_NUM_AXIS_GR; j++){
                    GroupAx[j].clear();
                 }
                 fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);
             }

             if(IsTruc == -1){
                fnAddSelectAxForSmallCar(Is_AxisFound, GroupAx);
             }

             if(Is_AxisFound){

             size_ax_sel = coord_bound_right - coord_bound_left + 1;
             std::vector<double> data_clean(size_ax_sel);
             std::vector<double> data;
             std::vector<CrackDetect> cracks;
             std::copy(data_latt + coord_bound_left,  data_latt + coord_bound_right, dat.data());
             for(std::vector<int32_t>::iterator it = dat.begin(); it != dat.end(); ++it){
                 data.push_back(1.0 * (*it));
             }
             remove_crack(data.begin(), data.end(), data_clean.begin(), coef_noise_latt, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight_lat, cracks, 5, 20.0, 0.8);

             CarAxisInfo ax_info;
             int num_ax = 0;
             int ax_base_coord = 0;
             if (GroupAx[0].size()!=0){
                 ax_base_coord = GroupAx[0].begin()->coord_ax_signal;
             }
             double weit_max = 0.;
             for(int i=0; i<MAX_NUM_AXIS_GR; i++){
                 for(std::vector<CarAxisDetect>::iterator it = GroupAx[i].begin(); it!=GroupAx[i].end(); ++it){
                     ax_info.num_ax = num_ax;
                     int ax_coord_beg_latt = it->coord_ax_signal + floor( 10.0 / car_info.speed * 5000 );
                     int ax_coord_end_latt = ax_coord_beg_latt + floor( 5.0 / car_info.speed * 5000 );
                     ax_info.abs_coord_beg_latt = ax_coord_beg_latt + coord_bound_left;
                     ax_info.abs_coord_end_latt = ax_coord_end_latt + coord_bound_left;

                     double weit_ax = 0.;
                     if (ax_coord_beg_latt >= data_clean.size()){
                         weit_ax = get_stdev(data_clean.data() + data_clean.size() - 1000,  data_clean.data() + data_clean.size() - 1);
                     }
                     else if(ax_coord_end_latt >= data_clean.size()){
                         weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt,  data_clean.data() + data_clean.size() - 1);
                     }
                     else{
                        weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt, data_clean.data() + ax_coord_end_latt);
                     }

                     ax_info.weit_ax_abs = fnGetCarWeit(ax_info.weit_axis, car_info.speed, GroupAx[i].size());
                     ax_info.weit_axis = weit_ax;
                     if(weit_ax > weit_max){
                         weit_max = weit_ax;
                     }
                     ax_info.coord_axis = 1.0 * (it->coord_ax_signal - ax_base_coord) / 5000.0 * car_info.speed * 1000.0;
                     num_ax++;
                 }
                 car_info.AxisInfo.push_back(ax_info);
             }

             if(weit_max > 4e5){
                 car_info.type = 1;
             }
             else if(weit_max < 2e5){
                 car_info.type = -1;
             }
             else{
                 car_info.type = 0;
             }

             int Yx = Y, Mx = M, Dx = D, hx = h, mx = m, sx = s, msx = ms;
             fnGetTime_for_SmplNum(car_info.AxisInfo[0].abs_coord_beg_latt, Yx, Mx, Dx, hx, mx,  sx,  msx, /*TStDat, size_all_tst */nullptr, 0);
             QDate date(Yx,Mx,Dx);
             QTime time(hx,mx,sx,msx);
             QDateTime date_time;
             car_info.date_time.setDate(date);
             car_info.date_time.setTime(time);

             cars_info.push_back(car_info);
             num_car++;
         }

         }
//выделение осей каждого обнаруженного автомобиля

    }
    else
    #pragma omp parallel
    {
#if defined(_OPENMP)
        std::vector<CarInfo> cars_info_priv;
#endif
        #pragma omp for
        for(int i=0; i < size_all - size_proc; i+=size_proc){

            if(progressBarHelper) progressBarHelper->emitProgress(round(static_cast<double>(size_proc)/(size_all - size_proc)*80));

            std::vector<int32_t> dat1(size_proc);
            std::vector<int32_t> dat2(size_proc);
            std::vector<double> signal_abs_flt_str(size_proc);
            std::vector<double> signal_abs_flt_lat(size_proc);

            std::vector<PulseGroup> pulse_groups_stript;
            std::vector<PulseGroup> pulse_groups_latt;
            std::vector<UnionPulseGroup> pulse_groups_un;

            std::copy(data_stript + i, data_stript + i + size_proc - 1, dat1.data());
            std::copy(data_latt + i, data_latt + i + size_proc - 1, dat2.data());

            std::thread th1(fnGetCarPowCenter, std::cref(dat1), 10000, 200, 100, 1024, 5000, 0, std::ref(pulse_groups_stript), std::ref(signal_abs_flt_str), 24743);
            std::thread th2(fnGetCarPowCenter, std::cref(dat2), 10000, 1000, 500, 1024, 5000, 1, std::ref(pulse_groups_latt), std::ref(signal_abs_flt_lat), 84763);

            th1.join();
            th2.join();

            // fnGetCarPowCenter(dat1, 10000, 200, 100, 1024, 5000, 0, pulse_groups_stript, signal_abs_flt_str, 24743);
            // fnGetCarPowCenter(dat2, 10000, 1000, 500, 1024, 5000, 1, pulse_groups_latt, signal_abs_flt_lat, 84763);

            fnUnionPulsesGroup(pulse_groups_stript, pulse_groups_latt, i, signal_abs_flt_lat, 5000, pulse_groups_un);

//выделение осей каждого обнаруженного автомобиля
            int num_car=0;
            for(std::vector<UnionPulseGroup>::iterator  it_pulse_groups=pulse_groups_un.begin(); it_pulse_groups!=pulse_groups_un.end(); ++it_pulse_groups){
                 std::vector<CarAxisDetect> GroupAx[MAX_NUM_AXIS_GR];
                 CarInfo car_info;

                 car_info.num_car = num_car;
                 car_info.speed = it_pulse_groups->freq * 0.25;
                 car_info.num_pulse_group = it_pulse_groups->num_pulse_group;


                 int coord_bound_left, coord_bound_right;
                 coord_bound_left = it_pulse_groups->pulses_latt.begin()->coord_pulse - floor( 15.0 / (it_pulse_groups->freq * 0.25) * 5000 ) - 7000;
                 coord_bound_left -= 500;
                 coord_bound_right = it_pulse_groups->pulses_latt.begin()->coord_pulse + floor( 10.0 / (it_pulse_groups->freq * 0.25) * 5000 ) + 5000;
                 coord_bound_right -=500;
                 int IsTruc = 0;
                 if(it_pulse_groups->pulses_latt.begin()->pow_pulse > 4.5e5){
                     IsTruc = 1;
                     coord_bound_left -= 5000;
                     coord_bound_right += 5000;
                 }
                 else if(it_pulse_groups->pulses_latt.begin()->pow_pulse < 2.5e5){
                    IsTruc = -1;
                 }
                 else{
                    coord_bound_left -= 3000;
                    coord_bound_right += 3000;
                 }

                 coord_bound_left += i;
                 coord_bound_right += i;

                 coord_bound_left = std::max(0, coord_bound_left);
                 const int tmp = size_all;
                 coord_bound_right = std::min(coord_bound_right, tmp);

                 int Shift = 0;

                 size_t size_ax_sel = coord_bound_right - coord_bound_left + 1;
                 std::vector<int32_t> dat(size_ax_sel);
                 std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());
                 fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);

                 if( Shift != 0){

                     if(Shift == 2){
                         coord_bound_left += 3000;
                         coord_bound_right -= 3000;
                     }
                     else if(Shift == 1){
                         coord_bound_left += 3000;
                     }
                     else if(Shift == -1){
                         coord_bound_right -= 3000;
                     }
                     else{}
                     coord_bound_left = std::max(0, coord_bound_left);
                     const int tmp = size_all;
                     coord_bound_right = std::min(coord_bound_right, tmp);

                     size_ax_sel = coord_bound_right - coord_bound_left + 1;
                     dat.resize(size_ax_sel);
                     std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());

                     for(int j=0; j<MAX_NUM_AXIS_GR; j++){
                        GroupAx[j].clear();
                     }

                     fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);
                 }

                 if(IsTruc == -1){
                    fnAddSelectAxForSmallCar(Is_AxisFound, GroupAx);
                 }

                 if(Is_AxisFound){

                 size_ax_sel = coord_bound_right - coord_bound_left + 1;
                 std::vector<double> data_clean(size_ax_sel);
                 std::vector<double> data;
                 std::vector<CrackDetect> cracks;
                 dat.resize(size_ax_sel);
                 std::copy(data_latt + coord_bound_left,  data_latt + coord_bound_right, dat.data());

                 for(std::vector<int32_t>::iterator it = dat.begin(); it != dat.end(); ++it){
                     data.push_back(1.0 * (*it));
                 }

                 remove_crack(data.begin(), data.end(), data_clean.begin(), coef_noise_latt, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight_lat, cracks, 5, 20.0, 0.8);

                 CarAxisInfo ax_info;
                 int num_ax=0;

                 int ax_base_coord=0;
                 if (GroupAx[0].size()!=0){
                     ax_base_coord = GroupAx[0].begin()->coord_ax_signal;
                 }
                 double weit_max = 0.;
                 for(int i=0; i<MAX_NUM_AXIS_GR; i++){
                     for(std::vector<CarAxisDetect>::iterator it = GroupAx[i].begin(); it!=GroupAx[i].end(); ++it){
                         ax_info.num_ax = num_ax;
                         int ax_coord_beg_latt = it->coord_ax_signal + floor( 10.0 / car_info.speed * 5000 );
                         int ax_coord_end_latt = ax_coord_beg_latt + floor( 5.0 / car_info.speed * 5000 );
                         ax_info.abs_coord_beg_latt = ax_coord_beg_latt + coord_bound_left;
                         ax_info.abs_coord_end_latt = ax_coord_end_latt + coord_bound_left;

                         double weit_ax = 0.;
                         if (ax_coord_beg_latt >= data_clean.size()){
                             weit_ax = get_stdev(data_clean.data() + data_clean.size() - 1000,  data_clean.data() + data_clean.size() - 1);
                         }
                         else if(ax_coord_end_latt >= data_clean.size()){
                             weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt,  data_clean.data() + data_clean.size() - 1);
                         }
                         else{
                            weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt, data_clean.data() + ax_coord_end_latt);
                         }

                         ax_info.weit_axis = weit_ax;
                         ax_info.weit_ax_abs = fnGetCarWeit(ax_info.weit_axis, car_info.speed, GroupAx[i].size());

                         if(weit_ax > weit_max){
                             weit_max = weit_ax;
                         }
                         ax_info.coord_axis = 1.0 * (it->coord_ax_signal - ax_base_coord) / 5000.0 * car_info.speed * 1000.0;
                         num_ax++;
                         car_info.AxisInfo.push_back(ax_info);
                    }
                 }

                 if(weit_max > 4e5){
                     car_info.type = 1;
                 }
                 else if(weit_max < 2e5){
                     car_info.type = -1;
                 }
                 else{
                     car_info.type = 0;
                 }

                 int Yx = Y, Mx = M, Dx = D, hx = h, mx = m, sx = s, msx = ms;
                 fnGetTime_for_SmplNum(car_info.AxisInfo[0].abs_coord_beg_latt, Yx, Mx, Dx, hx, mx,  sx,  msx, /*TStDat, size_all_tst*/ nullptr, 0);
                 QDate date(Yx,Mx,Dx);
                 QTime time(hx,mx,sx,msx);
                 QDateTime date_time;
                 car_info.date_time.setDate(date);
                 car_info.date_time.setTime(time);
#if defined(_OPENMP)
                 cars_info_priv.push_back(car_info);
#else
                 cars_info.push_back(car_info);
#endif
                 num_car++;
             }

             }
//выделение осей каждого обнаруженного автомобиля
       }
#if defined(_OPENMP)
        #pragma omp for ordered
        for(int i=0; i< omp_get_num_threads(); i++) {
        #pragma omp ordered
             cars_info.insert(cars_info.end(), cars_info_priv.begin(), cars_info_priv.end());
        }
#endif

       if(size_all % size_proc > 0){
            int i = ( size_all / size_proc ) * size_proc;
            size_t size_rem = size_all % size_proc;

            std::vector<int32_t> dat(size_rem);
            std::vector<double> signal_abs_flt_str(size_rem);
            std::vector<double> signal_abs_flt_lat(size_rem);

            std::vector<PulseGroup> pulse_groups_stript;
            std::vector<PulseGroup> pulse_groups_latt;
            std::vector<UnionPulseGroup> pulse_groups_un;

            std::copy(data_stript + i, data_stript + size_all - 1, dat.data());
            fnGetCarPowCenter(dat, 10000, 200, 100, 1024, 5000, 0, pulse_groups_stript, signal_abs_flt_str, 24743);

            std::copy(data_latt + i, data_latt + size_all - 1, dat.data());
            fnGetCarPowCenter(dat, 10000, 1000, 500, 1024, 5000, 1, pulse_groups_latt, signal_abs_flt_lat, 84763);

            fnUnionPulsesGroup(pulse_groups_stript, pulse_groups_latt, i, signal_abs_flt_lat, 5000, pulse_groups_un);

//выделение осей каждого обнаруженного автомобиля
            int num_car=0;
            for(std::vector<UnionPulseGroup>::iterator  it_pulse_groups=pulse_groups_un.begin(); it_pulse_groups!=pulse_groups_un.end(); ++it_pulse_groups){
                 std::vector<CarAxisDetect> GroupAx[MAX_NUM_AXIS_GR];
                 CarInfo car_info;

                 car_info.num_car = num_car;
                 car_info.speed = it_pulse_groups->freq * 0.25;
                 car_info.num_pulse_group = it_pulse_groups->num_pulse_group;

                 int coord_bound_left, coord_bound_right;
                 coord_bound_left = it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].coord_pulse - floor( 15.0 / (it_pulse_groups->freq * 0.25) * 5000 ) - 7000;
                 coord_bound_left -= 500;
                 coord_bound_right = it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].coord_pulse + floor( 10.0 / (it_pulse_groups->freq * 0.25) * 5000 ) + 5000;
                 coord_bound_right -= 500;
                 int IsTruc = 0;
                 if(it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].pow_pulse > 4e5){
                     IsTruc = 1;
                     coord_bound_left -= 5000;
                     coord_bound_right += 5000;
                 }
                 else if(it_pulse_groups->pulses_latt[it_pulse_groups->num_pulses_max_freq_stat].pow_pulse < 2e5){
                    IsTruc = -1;
                 }
                 else{
                    coord_bound_left -= 3000;
                    coord_bound_right += 3000;
                 }

                 coord_bound_left += i;
                 coord_bound_right += i;

                 coord_bound_left = std::max(0, coord_bound_left);
                 const int tmp = size_all;
                 coord_bound_right = std::min(coord_bound_right, tmp);

                 int Shift = 0;

                 size_t size_ax_sel = coord_bound_right - coord_bound_left + 1;
                 std::vector<int32_t> dat(size_ax_sel);
                 std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());
                 fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);

                 if( Shift != 0 ){

                     if(Shift == 2){
                         coord_bound_left += 3000;
                         coord_bound_right -= 3000;
                     }
                     else if(Shift == 1){
                         coord_bound_left += 3000;
                     }
                     else if(Shift == -1){
                         coord_bound_right -= 3000;
                     }
                     else{}
                     coord_bound_left = std::max(0, coord_bound_left);
                     const int tmp = size_all;
                     coord_bound_right = std::min(coord_bound_right, tmp);

                     size_ax_sel = coord_bound_right - coord_bound_left + 1;
                     dat.resize(size_ax_sel);
                     std::copy(data_stript + coord_bound_left,  data_stript + coord_bound_right, dat.data());
                     for(int j=0; j<MAX_NUM_AXIS_GR; j++){
                        GroupAx[j].clear();
                     }
                     fnSelectAxisGroupRes(dat, 600, 25, 25, 180, 180, IsTruc, Is_AxisFound, GroupAx, Shift);
                 }

                 if(IsTruc == -1){
                    fnAddSelectAxForSmallCar(Is_AxisFound, GroupAx);
                 }

                 if(Is_AxisFound){

                 size_ax_sel = coord_bound_right - coord_bound_left + 1;
                 std::vector<double> data_clean(size_ax_sel);
                 std::vector<double> data;
                 std::vector<CrackDetect> cracks;
                 std::copy(data_latt + coord_bound_left,  data_latt + coord_bound_right, dat.data());
                 for(std::vector<int32_t>::iterator it = dat.begin(); it != dat.end(); ++it){
                     data.push_back(1.0 * (*it));
                 }

                 remove_crack(data.begin(), data.end(), data_clean.begin(), coef_noise_latt, coef_b_60_130, coef_a_60_130, coef_noise_strip_hamming_weight_lat, cracks, 5, 20.0, 0.8);

                 CarAxisInfo ax_info;
                 int num_ax=0;
                 int ax_base_coord=0;
                 if (GroupAx[0].size()!=0){
                     ax_base_coord = GroupAx[0].begin()->coord_ax_signal;
                 }
                 double weit_max = 0.;
                 for(int i=0; i<MAX_NUM_AXIS_GR; i++){
                     for(std::vector<CarAxisDetect>::iterator it = GroupAx[i].begin(); it!=GroupAx[i].end(); ++it){
                         ax_info.num_ax = num_ax;
                         int ax_coord_beg_latt = it->coord_ax_signal + floor( 10.0 / car_info.speed * 5000 );
                         int ax_coord_end_latt = ax_coord_beg_latt + floor( 5.0 / car_info.speed * 5000 );
                         ax_info.abs_coord_beg_latt = ax_coord_beg_latt + coord_bound_left;
                         ax_info.abs_coord_end_latt = ax_coord_end_latt + coord_bound_left;

                         double weit_ax = 0.;
                         if (ax_coord_beg_latt >= data_clean.size()){
                             weit_ax = get_stdev(data_clean.data() + data_clean.size() - 1000,  data_clean.data() + data_clean.size() - 1);
                         }
                         else if(ax_coord_end_latt >= data_clean.size()){
                             weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt,  data_clean.data() + data_clean.size() - 1);
                         }
                         else{
                            weit_ax = get_stdev(data_clean.data() + ax_coord_beg_latt, data_clean.data() + ax_coord_end_latt);
                         }

                         ax_info.weit_axis = weit_ax;
                         ax_info.weit_ax_abs = fnGetCarWeit(ax_info.weit_axis, car_info.speed, GroupAx[i].size());

                         if(weit_ax > weit_max){
                             weit_max = weit_ax;
                         }
                         ax_info.coord_axis = 1.0 * (it->coord_ax_signal - ax_base_coord) / 5000.0 * car_info.speed * 1000.0;
                         num_ax++;
                         car_info.AxisInfo.push_back(ax_info);
                    }
                 }
                 if(weit_max > 4e5){
                     car_info.type = 1;
                 }
                 else if(weit_max < 2e5){
                     car_info.type = -1;
                 }
                 else{
                     car_info.type = 0;
                 }

                 int Yx = Y, Mx = M, Dx = D, hx = h, mx = m, sx = s, msx = ms;
                 fnGetTime_for_SmplNum(car_info.AxisInfo[0].abs_coord_beg_latt, Yx, Mx, Dx, hx, mx,  sx,  msx, /*TStDat, size_all_tst*/ nullptr, 0);
                 QDate date(Yx,Mx,Dx);
                 QTime time(hx,mx,sx,msx);
                 QDateTime date_time;
                 car_info.date_time.setDate(date);
                 car_info.date_time.setTime(time);

                 cars_info.push_back(car_info);
                 num_car++;
             }

             }
//выделение осей каждого обнаруженного автомобиля
       }
   }

    if(progressBarHelper) emit progressBarHelper->progress(100);
}

void fnUnionPulsesGroup(std::vector<PulseGroup> pulse_groups_stript, std::vector<PulseGroup> pulse_groups_latt, int shift_from_begin, std::vector<double> signal_abs_flt, int thDist, std::vector<UnionPulseGroup> & pulse_groups_un)
{
    int count = 0;
    int coord_str, coord_lat;

    for(std::vector<PulseGroup>::iterator  it_pulse_groups_st=pulse_groups_stript.begin(); it_pulse_groups_st!=pulse_groups_stript.end(); ++it_pulse_groups_st){
        UnionPulseGroup un_p_gr;
        un_p_gr.num_pulse_group = it_pulse_groups_st->num_pulse_group;

        if(count == 30){
            int err = 1;
        }

        for(std::vector<Pulses>::iterator it = it_pulse_groups_st->pulses.begin(); it !=it_pulse_groups_st->pulses.end(); ++it){
            un_p_gr.pulses_stript.push_back(*it);
        }

        un_p_gr.freq = 0;
        un_p_gr.freq_stat = 0;
        un_p_gr.IsLattAfterStript = false;

        coord_str=it_pulse_groups_st->pulses.begin()->coord_pulse;
        std::vector<PulseGroup>::iterator it_pulse_groups_lt = pulse_groups_latt.begin();
        coord_lat = it_pulse_groups_lt->pulses[it_pulse_groups_lt->num_pulses_max_freq_stat].coord_pulse;
        bool fl_break = false;
        while((coord_lat < coord_str)&&(it_pulse_groups_lt!=pulse_groups_latt.end())){
            ++it_pulse_groups_lt;
            if(it_pulse_groups_lt == pulse_groups_latt.end()){
                fl_break = true;
                break;
            }
            coord_lat = it_pulse_groups_lt->pulses[it_pulse_groups_lt->num_pulses_max_freq_stat].coord_pulse;
        }

        bool IsFindLattAfterStript = false;
        if(!fl_break){
            for(std::vector<Pulses>::iterator it_pulses_st = it_pulse_groups_st->pulses.begin(); it_pulses_st != it_pulse_groups_st->pulses.end(); ++it_pulses_st){
                coord_str = it_pulses_st->coord_pulse;
                for(std::vector<Pulses>::iterator it_pulses_lt = it_pulse_groups_lt->pulses.begin(); it_pulses_lt != it_pulse_groups_lt->pulses.end(); ++it_pulses_lt){
                    coord_lat = it_pulses_lt->coord_pulse;
                    if(coord_lat - coord_str < thDist){
                        IsFindLattAfterStript = true;
                        break;
                    }
                }
                if(IsFindLattAfterStript){
                    break;
                }
            }
        }
        if((IsFindLattAfterStript)&&(!it_pulse_groups_lt->IsSelect)){
            it_pulse_groups_lt->IsSelect = true;
            it_pulse_groups_st->IsSelect = true;
            un_p_gr.IsLattAfterStript = true;
            un_p_gr.freq = it_pulse_groups_lt->freq;
            un_p_gr.freq_stat = it_pulse_groups_lt->freq_stat;
            un_p_gr.num_pulses_max_freq_stat = it_pulse_groups_lt->num_pulses_max_freq_stat;

            for(std::vector<Pulses>::iterator it = it_pulse_groups_lt->pulses.begin(); it !=it_pulse_groups_lt->pulses.end(); ++it){
                un_p_gr.pulses_latt.push_back(*it);
            }

            pulse_groups_un.push_back(un_p_gr);
            count++;
        }
        else if(!IsFindLattAfterStript){
            std::vector<Pulses> ps_latt;
            Pulses p_latt;
            const int tmp = coord_str + floor( 12.5 / 20.0 * 5000 );
            p_latt.coord_pulse = std::min(WIN_SMP_PROCESS-1, tmp);
            p_latt.pow_pulse = signal_abs_flt[p_latt.coord_pulse];
            ps_latt.push_back(p_latt);
            un_p_gr.freq = 20.0 / 0.25;
            un_p_gr.num_pulses_max_freq_stat = 0;

            for(std::vector<Pulses>::iterator it = ps_latt.begin(); it !=ps_latt.end(); ++it){
                un_p_gr.pulses_latt.push_back(*it);
            }

            pulse_groups_un.push_back(un_p_gr);
            count++;
        }
        else{}
    }
}

void fnGetSmplNum_for_Timestr(int Y,  int M, int D, int h, int m,  int s,  int ms, const int64_t * TSt, int NumTSt, int & NB, int & ND){

    ND = 0;

    QTime time(h,m,s,ms);
    QDate date(Y, M, D);
    QDateTime date_time(date,time);
    int64_t time_unix = date_time.toMSecsSinceEpoch();

    int64_t DLTmin = 1e8;
    for(int i = 0; i<NumTSt; i++){
        int64_t next = int64_t(double(uint64_t(TSt[i])));
        int64_t t_u = int64_t(double(time_unix));
        int64_t DLT = int64_t(std::abs(t_u-next));
        if(DLT<=DLTmin){
            DLTmin = DLT;
            NB = i;
            ND=( i - 1 ) * 32 + 1;
        }
        else{
            break;
        }
    }
}

void fnGetTime_for_SmplNum(int ND, int & Y,  int & M, int & D, int & h, int & m,  int & s,  int & ms, const int64_t * TSt, int NumTSt){
    int NBx_0, NDx_0, NBx_1, NDx_1;
    int Yx = Y, Mx = M, Dx = D, hx = h, mx = m, sx = s, msx = ms;
    fnGetSmplNum_for_Timestr(Yx,  Mx, Dx, hx, mx, sx, msx, TSt, NumTSt, NBx_0, NDx_0);
    fnGetSmplNum_for_Timestr(Yx,  Mx, Dx, hx, mx, sx+1, msx, TSt, NumTSt, NBx_1, NDx_1);
//    int DD = NDx_1 - NDx_0;
    int DD = 5000;
    double ss = 1. * ( ND - NDx_0 ) / DD;
    int mss = ( ss - floor(ss) ) * 1000;
    ms += mss;
    if(ms >= 1000){
        ms-=1000;
        ss++;
    }
    int mm = floor( ss ) / 60;
    s += ( floor( ss ) - mm * 60 );
    if(s >= 60){
        s -= 60;
        mm++;
    }
    int hh = floor( mm / 60 );
    m += ( mm - hh * 60 );
    if(m >= 60){
        m -= 60;
        hh++;
    }

    h += hh;
}

void fnAddSelectAxForSmallCar(bool & Is_AxisFound, std::vector<CarAxisDetect> * GroupAx)
{
    Is_AxisFound = true;
    std::vector<CarAxisDetect> GroupAx_Sum;
    for(int i=0; i<MAX_NUM_AXIS_GR; i++){
        for(std::vector<CarAxisDetect>::iterator it = GroupAx[i].begin(); it!=GroupAx[i].end(); ++it){
            GroupAx_Sum.push_back(*it);
       }
    }

    if(GroupAx_Sum.size() > 2){
        std::vector<CarAxisDetect> GroupAx_Pair;
        double sum_pow_max = 0;
        double pow_max_0, pow_max_1;
        int coord_ax_0 = -1, coord_ax_1 = -1;
        for(std::vector<CarAxisDetect>::iterator it_ax_0 = GroupAx_Sum.begin();  it_ax_0 != GroupAx_Sum.end(); ++it_ax_0 ){
            for(std::vector<CarAxisDetect>::iterator it_ax_1 = GroupAx_Sum.begin();  it_ax_1 != GroupAx_Sum.end(); ++it_ax_1 ){
                if(std::abs(it_ax_0->coord_ax_signal - it_ax_1->coord_ax_signal) > 400){
                    if(it_ax_0->pow_ax_signal + it_ax_1->pow_ax_signal > sum_pow_max){
                        coord_ax_0 = it_ax_0->coord_ax_signal;
                        coord_ax_1 = it_ax_1->coord_ax_signal;
                        pow_max_0 = it_ax_0->pow_ax_signal;
                        pow_max_1 = it_ax_1->pow_ax_signal;
                        sum_pow_max = it_ax_0->pow_ax_signal + it_ax_1->pow_ax_signal;
                    }
                }
            }
        }
        GroupAx_Sum.clear();
        if((coord_ax_0 != -1)&&(coord_ax_1 != -1)){
            CarAxisDetect ax;
            ax.coord_ax_signal = coord_ax_0;
            ax.pow_ax_signal = pow_max_0;
            GroupAx_Sum.push_back(ax);
            ax.coord_ax_signal = coord_ax_1;
            ax.pow_ax_signal = pow_max_1;
            GroupAx_Sum.push_back(ax);
        }
        else{
            Is_AxisFound = false;
            return;
        }
        for(int i = 0; i<MAX_NUM_AXIS_GR; i++){
            GroupAx[i].clear();
        }
        GroupAx[0].push_back(GroupAx_Sum[0]);
        GroupAx[1].push_back(GroupAx_Sum[1]);
    }
    else if(GroupAx_Sum.size() <= 1){
        Is_AxisFound = false;
        return;
    }
    else{}
}

double fnGetCarWeit(double Amplitude, double Speed, size_t SizeGroup, double St, double AmplitudeNorm, double SpeedNorm)
{
    double Weit = AmplitudeNorm * std::pow(Amplitude * (2. - Speed / SpeedNorm), St) / std::pow(1.0 * SizeGroup, 0.5);
    return Weit;
}
