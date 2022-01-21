// ParticleFilter.cpp :
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string.h>
#include <pthread.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "define.h"
#include "Target.h"
#include "Laser.h"
#include "Particle_filter.h"

#include "Multiple_particle_filter.h"

extern pthread_mutex_t mutex_laser;
extern pthread_mutex_t mutex_target;

CMultipleParticleFilter::CMultipleParticleFilter()
{
    m_max_ID = 100;
    m_min_distance = 1200;//1000.0;  // 1000mm
    //m_min_distance = 560.0;   // 400mm
    m_ID = 0;
}

CMultipleParticleFilter::~CMultipleParticleFilter()
{
    m_ParticleFilter.clear();
}

void CMultipleParticleFilter::update(CLaser *Laser)
{
    m_pLaser = Laser;
    double obs[2], p[2];

    for (int n = 0; n < 1; n++)
    {
        //    for (int n = 0; n < m_pLaser->m_cnMaxConnect; n++)
        //  {
        if (m_pLaser->m_bNodeActive[n])
        {
            std::vector<int> label(m_pLaser->m_LRFClsPoints[n].size(), -1);     //
            int pn = m_ParticleFilter.size();   //
            //std::cout << "m_ParticleFilter.size" << m_ParticleFilter.size() << std::endl;

            for (int j = 0; j < m_pLaser->m_LRFClsPoints[n].size(); j++)
            {
                obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
                obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;

                double min_r = 1e10;
                int np = 0;
                for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np)
                {
                    p[0] = it->state[0];
                    p[1] = it->state[1];

                    double r = sqrt(pow(p[0] - obs[0], 2) + pow(p[1] - obs[1], 2));
                    //std::cout << "n__" << n << "r__" << r << std::endl;

                    if (min_r > r && r < m_min_distance)
                    {
                        min_r = r;
                        label[j] = np;
                    }
                }

                if (min_r >= m_min_distance)
                {
                    label[j] = pn++;
                }
            }

            //////////////////////////atode printf siteemiru

            std::vector<int> flg(pn, -1);       //
            for (int j = 0; j < m_pLaser->m_LRFClsPoints[n].size(); j++)
            {
                if (label[j] < 0)
                {
                    std::cout << "Error" << std::endl;
                }
                else if (label[j] < m_ParticleFilter.size())
                {
                    obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
                    obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;
                    m_ParticleFilter[label[j]].SetTarget(obs);
                    m_ParticleFilter[label[j]].update();
                }
                else
                {
                    CPF pf;
                    int area[2] = { STAGE_X, STAGE_Y };
                    pf.initialize(area);

                    obs[0] = m_pLaser->m_LRFClsPoints[n][j].x;
                    obs[1] = m_pLaser->m_LRFClsPoints[n][j].y;
                    pf.SetTarget(obs);
                    pf.SetID(m_ID++);
                    pf.update();
                    m_ParticleFilter.push_back(pf);
                }
                flg[label[j]] = 1;
            }

            int np = 0;
            for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np)
            {
                if (flg[np] < 0)
                {
                    it->clear();
                    it = m_ParticleFilter.erase(it);
                    if (it == m_ParticleFilter.end()) break;
                }
            }

            for (int i = 0; i < MAX_TRACKING_OBJECT; i++)
            {
                delete m_pLaser->m_pTarget[i];
                m_pLaser->m_pTarget[i] = NULL;
            }

            np = 0;
            for (vector<CPF>::iterator it = m_ParticleFilter.begin(); it != m_ParticleFilter.end(); ++it, ++np)
            {
                m_pLaser->m_pTarget[np] = new CTarget();
                m_pLaser->m_pTarget[np]->id = it->GetID();
                m_pLaser->m_pTarget[np]->cnt = it->GetCnt();
                m_pLaser->m_pTarget[np]->SetPosi(it->state[0], it->state[1]);
            }
            //std::cout << "np" << np << std::endl;
        }
    }
}
