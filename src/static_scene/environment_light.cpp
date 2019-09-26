#include "environment_light.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>


namespace CMU462 {
    namespace StaticScene {

        size_t upper_bound(double* array, size_t start, size_t end, double value)
//this is basically a binary search for finding index
        {
          size_t index = end;
          size_t L = start;
          size_t R = end;
          while (L < R)
          {
            size_t m = floor((L + R) / 2);
            if (array[m] < value && array[m+1] < value)
            {
              L = m + 1;
            }
            else if (array[m] > value && array[m+1] > value)
            {
              R = m - 1;
            }else
            {
              index = m;
              return index;
            }
          }

          return L;
        }

        EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
                : envMap(envMap) {
          // TODO: (PathTracer) initialize things here as needed
          // precomputing the joint distributions p(phi, theta) and
          // marginal distributions p(theta) in the constructor of EnvironmentLight
          // and storing the resulting values in fields of the class.

          //compute p(phi, theta) (joint distribution)
          size_t h = envMap->h;
          size_t w = envMap->w;
          prob = new double[h * w];
          margin = new double[h];
          double sum;

          for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
              size_t index = j + i * w;
              Spectrum color = envMap->data[index];
              double theta = i * PI / h;
              prob[index] = color.illum() * sin(theta);
              sum += prob[index];
            }
          }

          //divide by sum, normalize pdf
          for (int index = 0; index < w * h; index++)
            prob[index] = prob[index] / sum;

          //compute p(theta) (marginal distribution (pdf))
          for (int i = 0; i < h; i++)
          {
            double margin_sum = 0;
            for (int j = 0; j < w; j++)
            {
              margin_sum += prob[ i * w + j];
            }
            margin[i] = margin_sum;
          }


          //compute = p(theta|phi)
          for (int i = 0; i < h; i++)
          {
            for (int j = 0; j < w; j++)
            {
              if (margin[i] != 0) prob[j + i * w] = prob[j + i * w] / margin[i];
            }
          }

          //compute margin_cdf
          margin_cdf = new double[h];
          margin_cdf[0] = margin[0];
          for (int i = 1; i < h; i++)
          {
            margin_cdf[i] = margin_cdf[i-1] + margin[i];
          }

          //compute P(phi|theta) cdf
          prob_cdf = new double [h * w];
          for (int i = 0; i < h; i++)
          {
            prob_cdf[w * i] = prob[w * i];
            for (int j = 1; j < w; j++)
            {
              prob_cdf[w * i + j] = prob_cdf[w * i + j - 1] + prob[w * i + j];
            }
          }

        }

        Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                            float* distToLight, float* pdf) const {
          // TODO: (PathTracer) Implement
          size_t w = envMap->w;
          size_t h = envMap->h;
          //Get a uniform 2D sample on [0,1]^2[0,1]
          double Xi1 = (double)(std::rand()) / RAND_MAX;
          double Xi2 = (double)(std::rand()) / RAND_MAX;

          size_t index_r = upper_bound(margin_cdf,0, h-1, Xi1);
          size_t index_c = upper_bound(prob_cdf,index_r * w, index_r * w + w - 1, Xi2);

          double theta = PI * ((double)index_r  / h);
          double phi = 2 * PI * ((double)index_c / w);

          double y = cos(theta);
          double x = sin(theta) * cos(phi);
          double z = sin(theta) * sin(phi);

          *wi = (Vector3D (x, y, z)).unit();
          *distToLight = INF_F;
          *pdf = phi * theta * w * h / (2 * PI * PI * sin(theta));

          Ray r = Ray (Vector3D(0,0,0), *wi, INF_D);
          return sample_dir(r);

          //return Spectrum();
        }

        Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
          // TODO: (PathTracer) Implement

          size_t w = envMap->w;
          size_t h = envMap->h;

          double theta = acos(r.d.y); // 0 <= theta <= pi
          double phi = PI + atan2(r.d.x, -r.d.z); // 0 <= phi <= 2pi
          double x_texture = phi  / (2 * PI) * w;
          double y_texture = theta / PI * h;

          int t0_x = (int)(floor(x_texture));
          if (t0_x < 0) t0_x = 0;
          int t1_x = (int)(ceil(x_texture));
          if (t1_x > w - 1) t1_x = w - 1;
          int t0_y = (int)(floor(y_texture));
          if (t0_y < 0) t0_y = 0;
          int t1_y = (int)(ceil(y_texture));
          if (t1_y > h - 1) t1_y = h - 1;

          Spectrum color[4];
          color[0] = envMap->data[w * t0_y + t0_x];
          color[1] = envMap->data[w * t0_y + t1_x];
          color[2] = envMap->data[w * t1_y + t0_x];
          color[3] = envMap->data[w * t1_y + t1_x];

          double u = phi / (2 * PI); //0 <= u <= 1
          double v = theta / PI; // 0 <= v <= 1

          return (1 - u)*(1 - v) * color[0] + u * (1 - v) * color[1] + v * (1 - u) * color[2] + u * v * color[3];
        }

    }  // namespace StaticScene
}  // namespace CMU462
