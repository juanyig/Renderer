#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;

namespace CMU462 {

    void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
      Vector3D z = Vector3D(n.x, n.y, n.z);
      Vector3D h = z;
      if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
        h.x = 1.0;
      else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
        h.y = 1.0;
      else
        h.z = 1.0;

      z.normalize();
      Vector3D y = cross(h, z);
      y.normalize();
      Vector3D x = cross(z, y);
      x.normalize();

      o2w[0] = x;
      o2w[1] = y;
      o2w[2] = z;
    }

// Diffuse BSDF //

    Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return albedo * (1.0 / PI);
    }

    Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // TODO (PathTracer):
      // Implement DiffuseBSDF
      *wi = sampler.get_sample(pdf);
      return f(wo, *wi);
    }

// Mirror BSDF //

    Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return 1.0/cos_theta(wo) * reflectance;
    }

    Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // TODO (PathTracer):
      // Implement MirrorBSDF
      reflect(wo, wi);
      *pdf = 1;
      return f(wo, *wi);
    }

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

    Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                      float* pdf) {
      // TODO (PathTracer):
      // Implement RefractionBSDF
      *pdf = 1;

      if (refract(wo, wi, ior)) return 1.0 / abs(wi->z) * transmittance;
      reflect(wo, wi);
      return Spectrum();
    }

// Glass BSDF //

    Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // TODO (PathTracer):
      // Compute Fresnel coefficient and either reflect or refract based on it.

      //all reflected, no refraction (same as mirror)
      bool refraction = refract(wo,wi,ior);
      if (!refraction)
      {
        reflect(wo, wi);
        *pdf = 1;
        return 1.0/abs(cos_theta(wo)) * reflectance;
      }

      double cos, n1, n2, R0, Fr;
      if (cos_theta(wo) > 0) cos = cos_theta(wo);
      else cos = -cos_theta(wo);

      n1 = cos_theta(wo) > 0 ? 1.0 : ior;
      n2 = cos_theta(wo) > 0 ? ior : 1.0;
      R0 = ((n1 - n2) * (n1 - n2)) / ((n1 + n2) * (n1 + n2));
      Fr = R0 + (1.0 - R0) * pow((1.0 - cos), 5);
      if (((double)(std::rand()) / RAND_MAX) < Fr)
      {
        reflect(wo,wi);
        *pdf = Fr;
        return Fr / cos * reflectance;
      }

      *pdf = 1 - Fr;
      double n1_2 = n1 * n1;
      double n2_2 = n2 * n2;
      return (n1_2 / n2_2) * (1-Fr) / abs(cos) * transmittance;
    }

    void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
      // TODO (PathTracer):
      // Implement reflection of wo about normal (0,0,1) and store result in wi.
      *wi = Vector3D(-wo.x, -wo.y, wo.z);
    }

    bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
      // TODO (PathTracer):
      // Use Snell's Law to refract wo surface and store result ray in wi.
      // Return false if refraction does not occur due to total internal reflection
      // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
      // ray entering the surface through vacuum.

      double n1_n2;
      if (wo.z > 0) n1_n2 = 1.0 / ior;
      else n1_n2 = ior;

      double sin = sin_theta(wo);
      if (n1_n2 * sin > 1) return false;

      double wi_z = sqrt(1 - (n1_n2) * (n1_n2) * (sin_theta2(wo)));
      if (wo.z > 0) wi_z = -wi_z;
      *wi = (Vector3D(-wo.x * n1_n2, -wo.y * n1_n2, wi_z)).unit();
      return true;
    }

// Emission BSDF //

    Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      *wi = sampler.get_sample(pdf);
      return Spectrum();
    }

}  // namespace CMU462
