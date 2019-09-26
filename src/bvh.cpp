#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>
#include <float.h>

using namespace std;

namespace CMU462 {
    namespace StaticScene {


        BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                           size_t max_leaf_size) {
          this->primitives = _primitives;

          // TODO (PathTracer):
          // Construct a BVH from the given vector of primitives and maximum leaf
          // size configuration. The starter code build a BVH aggregate with a
          // single leaf node (which is also the root) that encloses all the
          // primitives.

          BBox bb;
          for (size_t i = 0; i < primitives.size(); ++i) {
            bb.expand(primitives[i]->get_bbox());
          }
          root = new BVHNode(bb, 0, primitives.size());
          partition(root, max_leaf_size);
        }

        bool compare_x (Primitive *a, Primitive *b)
        {
          return a->get_bbox().centroid()[0] < b->get_bbox().centroid()[0];
        }

        bool compare_y (Primitive *a, Primitive *b)
        {
          return a->get_bbox().centroid()[1] < b->get_bbox().centroid()[1];
        }

        bool compare_z (Primitive *a, Primitive *b)
        {
          return a->get_bbox().centroid()[2] < b->get_bbox().centroid()[2];
        }

        void BVHAccel::partition(BVHNode* node,
                                 size_t max_leaf_size) {

          //base: leaf nodes
          if (node->range <= max_leaf_size) return;

          //recursive: interior nodes

          //set buckets numbers, minimum cost, the axis of minimum cost(our actual) division, and
          //range of primitives that belong to the left node after minimum cost division
          size_t NUM = 16;
          double start = node->start;
          double range = node->range;
          double minCost_all_axis = range;
          int axis = 0;
          size_t left_range = 0;
          BBox boxl, boxr;

          //For each axis: x,y,z:
          for (int i = 0; i < 3; i++)
          {
            //if dimension == 0 (ex: trigs5.dae has an axis dimension of 0), skip to next axis
            if(node->bb.extent[i] != 0)
            {
              //initialize buckets
              BBox buckets_bbox[NUM];
              for (int j = 0; j < NUM;j++) buckets_bbox[i] = BBox();

              //sort primitive array

              if (i == 0)
                std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_x);
              if (i == 1)
                std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_y);
              if (i == 2)
                std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_z);

              //for each primitive p in node:
              // ->compute bucket
              // ->expand bucket bbox to make sure primitive partition, not spatial partition
              // ->update bucket's primitive array
              for (int j = start; j < start + range;j++)
              {
                Primitive *p = primitives[j];
                BBox b = p->get_bbox();
                Vector3D centriod = b.centroid();
                int index = (int)((centriod[i] - node->bb.min[i]) / (node->bb.extent[i] / NUM));
                // edge case: for example, if a primitive is parallel to y axis, its bounding box is a flat plane
                //where max, min, and centroid of y component will all be 0. If such primitive is at the boundary of its parent
                //bounding box on x (or z) axis, it will fall into bucket index = NUM (since centroid[x] - bb.max[x] = 0)
                if(index < 0) index = 0;
                if(index > NUM - 1) index = NUM - 1;

                buckets_bbox[index].expand(b);
                buckets_bbox[index].range += 1;
              }

              //determine (bucket size - 1) possible partitioning planes
              BBox left[NUM - 1];
              size_t left_prim_size[NUM-1];
              BBox right[NUM - 1];
              size_t right_prim_size[NUM-1];
              double cost[NUM-1];
              for (int k = 1; k < NUM; k++)
              {
                //initialize size_t array to 0
                left_prim_size[k - 1] = 0;
                for (int m = 0; m < k; m++)
                {
                  left[k - 1].expand(buckets_bbox[m]);
                  left_prim_size[k - 1] += buckets_bbox[m].range;
                }
                //initialize size_t array to 0
                right_prim_size[k - 1] = 0;
                for (int n = k; n < NUM; n++)
                {
                  right[k - 1].expand(buckets_bbox[n]);
                  right_prim_size[k - 1] += buckets_bbox[n].range;
                }

                //use SAH to determine cost = Sa * Na + Sb * Nb for each possible partition planes
                cost[k-1] = (left[k - 1].surface_area()/node->bb.surface_area()) * left_prim_size[k - 1] +
                            (right[k - 1].surface_area()/node->bb.surface_area()) * right_prim_size[k - 1];
              }

              //find min cost along all axis
              for (int k = 0; k < NUM - 1; k++)
              {
                if (cost[k] < minCost_all_axis)
                {
                  minCost_all_axis = cost[k];
                  left_range = left_prim_size[k];
                  axis = i;
                  boxl = left[k];
                  boxr = right[k];
                }
              }
            }
          }

          //perform actual partition (minimum cost along all 3 axis)
          //->rearrange primitive array if not edge case
          if (0 == left_range || range == left_range)
          {
            left_range = range / 2;
          } else
          {
            if (axis == 0)
              std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_x);
            if (axis == 1)
              std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_y);
            if (axis == 2)
              std::sort(primitives.begin()+start, primitives.begin()+start+range, compare_z);
          }
          //->create child nodes, then do recursion
          node->l = new BVHNode(boxl, start, left_range);
          node->r = new BVHNode(boxr, start + left_range, range - left_range);

          partition(node->l, max_leaf_size);
          partition(node->r, max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
          // TODO (PathTracer):
          // Implement a proper destructor for your BVH accelerator aggregate
          root->delete_tree(root);
        }

        void BVHNode::delete_tree(BVHNode *root)
        {
          if (root->l != nullptr) delete_tree(root->l);
          if (root->r != nullptr) delete_tree(root->r);
          delete root;
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        bool BVHAccel::intersect_node(BVHNode* node, const Ray &ray) const {
          double t0, t1 = 0.0;
          if (!node->bb.intersect(ray, t0, t1)) return false;

          bool hit = false;
          if (node->isLeaf()) {
            for (int i = node->start; i < node->start + node->range; i++) {
              if (primitives[i]->intersect(ray)) hit = true;
            }
            return hit;
          } else {
            double l0, l1, r0, r1 = 0.0;
            bool hit1 = false;
            if (node->l != nullptr && node->l->bb.intersect(ray, l0, l1)) hit1 = intersect_node(node->l, ray);
            bool hit2 = false;
            if (node->r != nullptr && node->r->bb.intersect(ray, r0, r1)) hit2 = intersect_node(node->r, ray);
            return hit1 || hit2;
          }
        }


        bool BVHAccel::intersect(const Ray &ray) const {
          // TODO (PathTracer):
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a BVH aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate.
          return intersect_node(root, ray);
        }

        bool BVHAccel::intersect_node(BVHNode* node, const Ray &ray, Intersection *isect) const{

          double t0, t1 = 0.0;
          if (!node->bb.intersect(ray, t0, t1)) return false;

          bool hit = false;
          if (node->isLeaf())
          {
            for (int i = node->start; i < node->start+node->range;i++)
            {
              if(primitives[i]->intersect(ray, isect)) hit = true;
            }
            return hit;
          }
          else
          {
            double l0,l1,r0,r1 = 0.0;
            bool hit1 = false;
            if (node->l != nullptr && node->l->bb.intersect(ray, l0, l1)) hit1 = intersect_node(node->l, ray, isect);
            bool hit2 = false;
            if (node->r != nullptr && node->r->bb.intersect(ray, r0, r1)) hit2 = intersect_node(node->r, ray, isect);
            return hit1 || hit2;
          }
        }

        bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {
          // TODO (PathTracer):
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a BVH aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate. When an intersection does happen.
          // You should store the non-aggregate primitive in the intersection data
          // and not the BVH aggregate itself.

          return intersect_node(root, ray, i);

        }

    }  // namespace StaticScene
}  // namespace CMU462
