#include "map2d.h"

Map2d::Map2d(double width, double height, double resolution) {
    resolution_ = resolution;
    width_ = width; 
    height_ = height;
    odds = std::vector<double>(width_*height_, 1.0);
    p_hit = 0.55;
    p_miss = 0.49;
    lofree = p_miss/(1-p_miss);
    looccu = p_hit/(1-p_hit);
}

void Map2d::bresenham(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2)
{
    //drawing a line from p1 to p2
    int dx = abs(p2(0) - p1(0));
    int sx = (p2(0) > p1(0)) ? 1 : -1;
    int dy = abs(p2(1) - p1(1));
    int sy = (p2(1) > p1(1)) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1(0);
    int y1 = p1(1);
    int x2 = p2(0);
    int y2 = p2(1);

    while (x1 != x2 && y1 != y2)
    {
        addMiss(y1, x1);
        cell2update_.insert(y1*width_+x1);
        if(isOccupied(y1*width_+x1)) {
            return;
        }
        int e2 = err;
        if (e2 > -dx) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
    addOccupied(p2(1), p2(0));
    cell2update_.insert(p2(1)*width_+p2(0));
}

void Map2d::update_map(const Eigen::Vector2i& origin, const CloudType::Ptr& scan_w)
{
    cell2update_.clear();
    //update map with scan and state
    // Eigen::Vector2i origin = world2map_i(state.t);
    if(origin(0) < 0 || origin(0) >= height_ || origin(1) < 0 || origin(1) >= width_) return;
    for (size_t i = 0; i < scan_w->size(); i++)
    {
        // pclPointType p_w = scan_w.points[i];
        // pclPointType p = scan.points[i];
        // float dist = sqrtf(p.x * p.x + p.y * p.y);
        // if (dist > 20) continue;

        Eigen::Vector2i pt = world2map_i(scan_w->points[i]);
        if (pt(0) < 0 || pt(0) >= height_ || pt(1) < 0 || pt(1) >= width_)
            continue;

        bresenham(origin, pt);
    }
    // cvmap2map();
}

int Map2d::scan_map_match_score(const CloudType::Ptr& scan ,const Eigen::Vector3d& pose) {
    Eigen::Matrix3d T = pose2Matrix3d(pose);
    CloudType::Ptr transformedScan = std::make_shared<CloudType>(scan->size());
    transfromCloudVec(scan, transformedScan, T);
    int score = 0;
    for (size_t i = 0; i < transformedScan->size(); i++) {
        Eigen::Vector2d& p = transformedScan->points[i];
        Eigen::Vector2d pp = world2map_d(p);
        //cout << "pp: " << pp.transpose() << endl;
        if((pp(0) <= 1) || (pp(0) >= height_) || (pp(1) <= 1) || (pp(1) >= width_)) {
            continue;
        } 
        else {
            //get value from map
            int x = round(pp(0));
            int y = round(pp(1));
            if(isOccupied(y*width_+x)) {
                score++;
            }
            //printf("i: %d, res:%f\n", i, residual[i]);
        }
    }
    //generate local map and compute local optimal?
    return score;
}

Eigen::Vector2i Map2d::world2map_i(const Eigen::Vector2d& p)
{
    Eigen::Vector2i m;
    m(0) = round(p(0) / resolution_ + width_ * 0.5);
    m(1) = round(p(1) / resolution_ + height_ * 0.5);
    return m;
}

Eigen::Vector2d Map2d::world2map_d(const Eigen::Vector2d& p)
{
    Eigen::Vector2d m;
    m = p / resolution_;
    m(0) += width_ * 0.5;
    m(1) += height_ * 0.5;
    return m;
}

inline void Map2d::cubic_weight(float xin, float a, float& w, float& dw)
{
    float x = fabs(xin);
    if (x <= 1) {
        w = 1 - (a + 3)*x*x + (a + 2)*x*x*x;
        if(xin>0) {
            dw = -2*(a+3)*xin+3*(a+2)*xin*xin;
        } else {
            dw = -2*(a+3)*xin-3*(a+2)*xin*xin;
        }
        return;
    }
    else if (x < 2) {
        w = -4 * a + 8 * a*x - 5 * a*x*x + a*x*x*x;
        if(xin>0) {
            dw = 8*a-10*a*xin+3*a*xin*xin;
        } else {
            dw = -8*a-10*a*xin-3*a*xin*xin;
        }
        return;
    }
    w = 0;  dw = 0;
}

double Map2d::bicubic_interpolation(const Eigen::Vector2d& p, Eigen::Vector2d& grad) {
    Eigen::Vector2d xy  = world2map_d(p);
    float x = xy(1);
    float y = xy(0);
    double a = -0.5;
    float coeff[16], grad_x[16], grad_y[16];
    //计算权重
    float u = x - floor(x) + 1;
    float v = y - floor(y) + 1;
    float col_w[4], col_dw[4];
    cubic_weight((u), a, col_w[0], col_dw[0]);
    cubic_weight((u - 1), a, col_w[1], col_dw[1]);
    cubic_weight((u - 2), a, col_w[2], col_dw[2]);
    cubic_weight((u - 3), a, col_w[3], col_dw[3]);
    for (int s = 0; s < 4; s++) {
        float row_w, row_dw;
        cubic_weight((v - s), a, row_w, row_dw);
        for(int c=0; c<4; c++) {
            coeff[s * 4 + c] = col_w[c] * row_w;
            grad_x[s * 4 + c] = col_dw[c] * row_w;
            grad_y[s * 4 + c] = col_w[c] * row_dw;
        }
    }
    //计算插值
    float sum = 0.0;
    int x0 = floor(x) - 1;
    int y0 = floor(y) - 1;
    grad.setZero();
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            double probability = getProbability((x0+i)*width_+y0+j);
            sum += coeff[i*4+j]*probability;
        }
    }
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            double probability = getProbability((x0+i)*width_+y0+j);
            grad(1) += grad_x[i*4+j]*(probability-sum);
            grad(0) += grad_y[i*4+j]*(probability-sum);
        }
    }
    return (double)sum;
}

/*
* 线性插值
*/
double Map2d::bilinearInterpolation(const Eigen::Vector2d& p, Eigen::Vector2d& grad) {
    Eigen::Vector2d xy = world2map_d(Eigen::Vector2d(p(1),p(0)));
    Eigen::Vector2i xy0(floor(xy(0)), floor(xy(1)));
    Eigen::Vector2d uv = xy - xy0.cast<double>();
    double value =  getProbability(xy0(0)*width_+xy0(1))*uv(0)*uv(1) + 
                    getProbability(xy0(0)*width_+xy0(1)+1)*(1-uv(0))*uv(1) + 
                    getProbability((xy0(0)+1)*width_+xy0(1))*uv(0)*(1-uv(1)) + 
                    getProbability((xy0(0)+1)*width_+xy0(1)+1)*(1-uv(0))*(1-uv(1));
    grad(1) = -0.5*((getProbability(xy0(0)*width_+xy0(1)+1)-getProbability(xy0(0)*width_+xy0(1)))*uv(1)+
            (getProbability((xy0(0)+1)*width_+xy0(1)+1)-getProbability((xy0(0)+1)*width_+xy0(1)))*(1-uv(1)));
    grad(0) = -0.5*((getProbability((xy0(0)+1)*width_+xy0(1))-getProbability(xy0(0)*width_+xy0(1)))*uv(0)+
            (getProbability((xy0(0)+1)*width_+xy0(1)+1)-getProbability(xy0(0)*width_+xy0(1)+1))*(1-uv(0)));
    return value;
}