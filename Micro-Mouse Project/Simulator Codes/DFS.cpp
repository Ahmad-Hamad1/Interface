#include <iostream>
#include <string>
#include <bits/stdc++.h>
#include "API.h"
using namespace std;
struct cell{
  int x,y;
  char direction, turn;
};

int arr[8][8] = {
  {6, 5, 4, 3, 3, 4, 5, 6},
  {5, 4, 3, 2, 2, 3, 4, 5},
  {4, 3, 2, 1, 1, 2, 3, 4},
  {3, 2, 1, 0, 0, 1, 2, 3},
  {3, 2, 1, 0, 0, 1, 2, 3},
  {4, 3, 2, 1, 1, 2, 3, 4},
  {5, 4, 3, 2, 2, 3, 4, 5},
  {6, 5, 4, 3, 3, 4, 5, 6}
};

int sizeX = 16, sizeY = 16;

int vis[20][20] = {0};
void log(const std::string& text) { std::cerr << text << std::endl; }

bool check(int x, int y, char direction, char turn) {
  if (direction == 'N') {
    if (turn == 'R') {
      x++;
    } else if (turn == 'L') {
      x--;
    } else {
      y++;
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      x--;
    } else if (turn == 'L') {
      x++;
    } else {
      y--;
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      y--;
    } else if (turn == 'L') {
      y++;
    } else {
      x++;
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      y++;
    } else if (turn == 'L') {
      y--;
    } else {
      x--;
    }
  }
if(sizeX > 16 || sizeY > 16 || sizeX < 0 || sizeY < 0) {
return 0;
}
  int yes = 0;
  log(std::to_string(x) + " " + std::to_string(y) + " " +
      std::to_string(vis[x][y]));
  if (vis[x][y] == 1) yes = 1;
  vis[x][y] = 1;
  return yes;
}
int getXY(int x, int y, char direction, char turn, int flag) {
  if (direction == 'N') {
    if (turn == 'R') {
      x++;
    } else if (turn == 'L') {
      x--;
    } else {
      y++;
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      x--;
    } else if (turn == 'L') {
      x++;
    } else {
      y--;
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      y--;
    } else if (turn == 'L') {
      y++;
    } else {
      x++;
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      y++;
    } else if (turn == 'L') {
      y--;
    } else {
      x--;
    }
  }
  return flag == 0 ? x : y;
}
char getNewDirection(char direction, char turn) {
  if (direction == 'N') {
    if (turn == 'R') {
      return 'E';
    } else if (turn == 'L') {
      return 'W';
    } else {
      return 'N';
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      return 'W';
    } else if (turn == 'L') {
      return 'E';
    } else {
      return 'S';
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      return 'S';
    } else if (turn == 'L') {
      return 'N';
    } else {
      return 'E';
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      return 'N';
    } else if (turn == 'L') {
      return 'S';
    } else {
      return 'W';
    }
  }
  return 'r';
}
void dfs(int x, int y, char direction, char turn) {
  if((x == 7 && y == 7) || (x == 7 && y == 8) || (x == 8 && y == 7) || (x == 8 && y == 8))
    exit(0);
  API::setColor(x, y, 'G');
  log(std::to_string(x) + " " + std::to_string(y));
  // right
  if (!API::wallRight() && !check(x, y, direction, 'R')) {
    API::turnRight();
    API::moveForward();
    dfs(getXY(x, y, direction, 'R', 0), getXY(x, y, direction, 'R', 1),
        getNewDirection(direction, 'R'), 'R');
  }
  // left
  if (!API::wallLeft() && !check(x, y, direction, 'L')) {
    API::turnLeft();
    API::moveForward();
    dfs(getXY(x, y, direction, 'L', 0), getXY(x, y, direction, 'L', 1),
        getNewDirection(direction, 'L'), 'L');
  }
  // forward
  if (!API::wallFront() && !check(x, y, direction, 'F')) {
    API::moveForward();
    dfs(getXY(x, y, direction, 'F', 0), getXY(x, y, direction, 'F', 1),
        getNewDirection(direction, 'F'), 'F');
  }
  API::turnLeft();
  API::turnLeft();
  API::moveForward();
  if (turn == 'R') {
    API::turnRight();
    return;
  }
  if (turn == 'L') {
    API::turnLeft();
    return;
  }
  if (turn == 'F' && !(x == 0 && y == 0)) {
    log("Turning from " + std::to_string(x) + " " + std::to_string(y));
    API::turnRight();
    API::turnRight();
    return;
  }
}
void move(char turn){
  // right
  if (turn == 'R') {
    API::turnRight();
    API::moveForward();
  }
  // left
  if (turn == 'L') {
    API::turnLeft();
    API::moveForward();
  }
  // forward
  if (turn == 'F') {
    API::moveForward();
  }
}
int main(int argc, char* argv[]) {
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "abc");
  dfs(0, 0, 'N', 'F');
}
