#include "KMeans.cpp"



class GMM:public KMeans
{
public:
//inherit
//    const int m_dimNum=3;
//    int m_clusterNum;
//    int m_maxIterNum;
//    double m_endError;
//    std::vector<Eigen::Vector3d> m_means;

    std::vector<double> m_priors;
    std::vector<Eigen::Matrix3d> m_vars;

    GMM(int clusterNum,int maxIterNum,double endError):KMeans(clusterNum,maxIterNum,endError){
        for (int i = 0; i < m_clusterNum; i++){
            m_priors.push_back(1.0 / m_clusterNum);
            Eigen::Matrix3d tmpA;
            tmpA<< 0.1,0,0,
                    0,0.1,0,
                    0,0,0.1;
            m_vars.push_back(tmpA);
        }
    };

    double getProbabilityDensity(int n, Eigen::Vector3d x){
        double p=1;
        p *= 1 / (pow(sqrt(2 * M_PI),m_dimNum)*sqrt(m_vars[n].determinant()));
        p *= exp(-0.5 * (x - m_means[n]).transpose() *m_vars[n].inverse()* (x - m_means[n]));
        return p;
    };

    int ClusterGMM(std::vector<Eigen::Vector3d> data) {
        printf("ClusterKMeans,data.size():%zu\n",data.size());
        ClusterKMeans(data);
        printf("ClusterGMM,m_clusterNum:%d\n",m_clusterNum);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> gamma(data.size(),m_clusterNum);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> buffergamma(data.size(),m_clusterNum+1);

        double lastdist=0,currdist = 0;
        int unchanged = 0,iterNum = 0;;
        bool loop=true;
        while(loop) {
            lastdist=currdist;
            for (int i = 0; i < data.size(); i++) {
                buffergamma(i,m_clusterNum)=0;
                for (int j = 0; j < m_clusterNum; j++) {
                    buffergamma(i,j) = m_priors[j]*getProbabilityDensity(j, data[i]);
                    buffergamma(i,m_clusterNum)+=buffergamma(i,j);
                }
            }

            for (int i = 0; i < data.size(); i++) {
                for (int j = 0; j < m_clusterNum; j++) {
                    gamma(i,j) = buffergamma(i,j)/buffergamma(i,m_clusterNum);
                }
            }
            for (int i = 0; i < m_clusterNum; i++) {
                double meandown = 0;
                Eigen::Vector3d meanup(0, 0, 0);
                Eigen::Matrix3d varup;
                varup << 0, 0, 0,
                        0, 0, 0,
                        0, 0, 0;
                for (int j = 0; j < data.size(); j++) {
                    meanup += gamma(j,i) * data[j];
                    meandown += gamma(j,i);
                }
                m_means[i] = meanup / meandown;
                for (int j = 0; j < data.size(); j++) {
                    varup += gamma(j,i) * (data[j] - m_means[i])*(data[j] - m_means[i]).transpose();
                }
                m_vars[i] = varup / meandown;
                m_priors[i] = meandown / data.size();
            }

            //LLD
            currdist=0;
            for(int i=0;i<data.size();i++){
                double tmpp=0;
                for(int j=0;j<m_clusterNum;j++){
                    tmpp+=m_priors[j]*getProbabilityDensity(j, data[i]);
                }
                currdist+=log(tmpp);
            }
            iterNum++;
            if (fabs(lastdist - currdist) < m_endError * lastdist){
                unchanged++;
            }
            if (iterNum >= m_maxIterNum || unchanged >= 3){
                loop = false;
            }
            printf("%d/%d,%lf\n",iterNum,m_maxIterNum,currdist);
        }
    }
};
