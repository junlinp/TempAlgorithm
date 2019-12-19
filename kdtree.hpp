#ifndef KDTREE_H_
#define KDTREE_H_
#include  <algorithm>
#include <cstring>
#include <vector>
#include <set>
#include <iostream>
template<class T, int Dimensional>
class KDTree {
 public:
  KDTree(const std::vector<T>& points):_root(new Node){

    std::vector<size_t> node_idx;
    size_t idx = 0;

    for(const T& point : points) {
      _data_set.push_back(point);
      node_idx.push_back(idx++);
    }

    MKTree(_root, node_idx, 0);


  }
  virtual ~KDTree() {
    NodeFree(_root);
  };

 private:
  struct Node {
    int idx = -1;
    int split_idx = -1;
    Node *_left = nullptr, *_right = nullptr;
  };

  void NodeFree(Node* node) {
    if (node) {
      NodeFree(node->_right);
      NodeFree(node->_left);
      delete node;
    }
  }
  void MKTree(Node* root,
      std::vector<size_t>& data_set_idx,
      size_t _split_dim) {

    if (root == nullptr) {
      return;
    }

    sort(data_set_idx.begin(), data_set_idx.end(), [this, &_split_dim](size_t lhs, size_t rhs) {
      return _data_set[lhs][_split_dim] < _data_set[rhs][_split_dim];
    });

    size_t size = data_set_idx.size();
    root->idx = data_set_idx[size / 2];
    root->split_idx = _split_dim;
    std::vector<size_t> lson_data_set, rson_data_set;
    for(size_t i =  0; i < size / 2; i++) {
      if (_data_set[data_set_idx[i]][_split_dim] < _data_set[root->idx][_split_dim]) {
        lson_data_set.emplace_back(data_set_idx[i]);
      } else {
        rson_data_set.emplace_back(data_set_idx[i]);
      }
    }
    for(size_t i =  size / 2 + 1; i < size; i++) {
      if (_data_set[data_set_idx[i]][_split_dim] < _data_set[root->idx][_split_dim]) {
        lson_data_set.emplace_back(data_set_idx[i]);
      } else {
        rson_data_set.emplace_back(data_set_idx[i]);
      }
    }

    if (lson_data_set.size()) {
      root->_left = new Node;
      MKTree(root->_left, lson_data_set, (_split_dim + 1) % Dimensional);
    }

    if (rson_data_set.size()) {
      root->_right = new Node;
      MKTree(root->_right, lson_data_set, (_split_dim + 1) % Dimensional);
    }


  }
  Node* _root;
  std::vector<T> _data_set;
};

struct PointXYZ {
  double x, y, z;
};
class RectTree {
 public:

  std::vector<PointXYZ> _point_set;
  std::vector<size_t> _x_index, _y_index;
  char* filter = nullptr;
  RectTree() = default;

  RectTree(const RectTree& rhs) {
    _point_set = rhs._point_set;
    _x_index = rhs._x_index;
    _y_index = rhs._y_index;
    filter = new char[_point_set.size()];
    memset(filter, 0, _point_set.size());
    std::cout << " CopyConstructor " << std::endl;
  }
 ~RectTree() {
   if (filter) {
     delete[] filter;
   }
 }

 void SetPoints(const std::vector<PointXYZ>& points) {
   _point_set = points;
   filter = new char[_point_set.size()];
   memset(filter, 0, _point_set.size());
   size_t size = _point_set.size();
   _x_index.resize(_point_set.size());
   _y_index.resize(size);

   for(size_t i = 0; i < size; i++) {
     _x_index[i] = _y_index[i] = i;
   }

   std::sort(_x_index.begin(), _x_index.end(), [this](size_t lhs, size_t rhs) {
      return this->_point_set[lhs].x < this->_point_set[rhs].x;
   });

   std::sort(_y_index.begin(), _y_index.end(), [this](size_t lhs, size_t rhs) {
     return this->_point_set[lhs].y < this->_point_set[rhs].y;
   });
 }

 std::vector<PointXYZ> Search(double x, double y, double radius) {
  std::vector<PointXYZ> res;
   auto x_functor =[this](size_t idx, double value) {
     return this->_point_set[idx].x < value;
   };
   auto y_functor = [this](size_t idx, double value) {
     return this->_point_set[idx].y < value;
   };

   auto x_lower_bound_iterator =
       std::lower_bound(_x_index.begin(), _x_index.end(), x - radius, x_functor);
   auto x_high_bound_iterator =
       std::lower_bound(_x_index.begin(), _x_index.end(), x + radius, x_functor);
   auto y_lower_bound_iterator =
       std::lower_bound(_y_index.begin(), _y_index.end(), y - radius, y_functor);
   auto y_high_bound_iterator =
       std::lower_bound(_y_index.begin(), _y_index.end(), y + radius, y_functor);


   for(auto it = x_lower_bound_iterator; it != x_high_bound_iterator; ++it) {
    filter[*it] = 1;
   }
   for(auto it = y_lower_bound_iterator; it != y_high_bound_iterator; ++it) {
     if(filter[*it]) {
        res.push_back(_point_set[*it]);
     }
   }
   for(auto it = x_lower_bound_iterator; it != x_high_bound_iterator; ++it) {
     filter[*it] = 0;
   }
   return res;
 }


};
#endif