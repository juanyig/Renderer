#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

    bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
      // TODO (PathTracer):
      // Implement ray - bounding box intersection test
      // If the ray intersected the bounding box within the range given by
      // t0, t1, update t0 and t1 with the new intersection times.
      double tmin, tmax,tmp;
      t0 = -INF_D;
      t1 = INF_D;
      for (int i = 0; i < 3; i++)
      {
        tmin = (min[i] - r.o[i]) / r.d[i];
        tmax = (max[i] - r.o[i]) / r.d[i];
        if (tmin > tmax)
        {
          tmp = tmin;
          tmin = tmax;
          tmax = tmp;
        }
        if (t0 > tmax || t1 < tmin) return false;
        if (t0 < tmin) t0 = tmin;
        if (t1 > tmax) t1 = tmax;
      }

      return true;
    }

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();
}

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
