/*
 * Definition of obstacleMAP class
 * this is just a collection of line segments
 *
 * Dominic Jonak
 */
#ifndef OBSTACLE_MAP_H
#define OBSTACLE_MAP_H

#include <rasm_common_types.h>

namespace TMAP {

  class obstacleList{
  private:
    struct node{
      int counter;
      RASM::point3d a, b;
      struct node *next;
    };
    node *head;
  public:
    obstacleList():head(NULL){}
    ~obstacleList();
    void insert(const RASM::point3d &a, const RASM::point3d &b, int count);
    bool crosses(const RASM::point3d &a, const RASM::point3d &b) const;
    void writeToFile(const char *filename)const;
    void readFromFile(const char *filename);
    void prune();
  }; /* end of class obstacleList */

  class obstacleMAP{
  private:
    obstacleList *obstacles;
  public:
    obstacleMAP();
    ~obstacleMAP();
    void inline insert(const RASM::point3d &a, const RASM::point3d &b, int count=-1){
      obstacles->insert(a, b, count);
    }
    bool inline crossesObstacle(const RASM::point3d &a, const RASM::point3d &b) const{
      return obstacles->crosses(a, b);
    }
    void inline writeToFile(const char *filename)const{
      obstacles->writeToFile(filename);
    }
    void inline readFromFile(const char *filename){
      obstacles->readFromFile(filename);
    }
    void inline prune() const{
      obstacles->prune();
    }

    /* removes all obstacles */
    void clearObstacles();

  };/* end of class obstacleMAP */

} /* end of namespace TMAP */

#endif /* end of OBSTACLE_MAP_H */
