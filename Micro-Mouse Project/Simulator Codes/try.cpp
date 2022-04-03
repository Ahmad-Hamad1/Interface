#include <bits/stdc++.h>

#include <iostream>
#include <string>

#include "API.h"
using namespace std;
struct cell {
  int x, y;
  char direction, turn;
};

int again = 1;

const int MAZE_SIZE_Y = 16;
const int MAZE_SIZE_X = 16;
int wall[MAZE_SIZE_X][MAZE_SIZE_Y][4] = {
    0};  // 0 : north, 1 : east, 2 : south, 3 : west

map<char, int> m;
int arr[MAZE_SIZE_X][MAZE_SIZE_Y];
int color[MAZE_SIZE_X][MAZE_SIZE_Y] = {0};
stack<pair<int,int>> path;
bool wallBackward() {
  API::turnLeft();
  API::turnLeft();
  bool wall = API::wallFront();
  API::turnLeft();
  API::turnLeft();
  return wall;
}

int vis[20][20] = {0};
void log(const std::string& text) { std::cerr << text << std::endl; }
int getXY(int x, int y, char direction, char turn, int flag) {
  if (direction == 'N') {
    if (turn == 'R') {
      x++;
    } else if (turn == 'L') {
      x--;
    } else if (turn == 'F') {
      y++;
    } else {
      y--;
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      x--;
    } else if (turn == 'L') {
      x++;
    } else if (turn == 'F') {
      y--;
    } else {
      y++;
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      y--;
    } else if (turn == 'L') {
      y++;
    } else if (turn == 'F') {
      x++;
    } else {
      x--;
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      y++;
    } else if (turn == 'L') {
      y--;
    } else if (turn == 'F') {
      x--;
    } else {
      x++;
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
    } else if (turn == 'F') {
      return 'N';
    } else {
      return 'S';
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      return 'W';
    } else if (turn == 'L') {
      return 'E';
    } else if (turn == 'F') {
      return 'S';
    } else {
      return 'N';
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      return 'S';
    } else if (turn == 'L') {
      return 'N';
    } else if (turn == 'F') {
      return 'E';
    } else {
      return 'W';
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      return 'N';
    } else if (turn == 'L') {
      return 'S';
    } else if (turn == 'F') {
      return 'W';
    } else {
      return 'E';
    }
  }
  return 'r';
}
void move(char turn) {
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

char inverseDirection(char direction) {
  if (direction == 'S') {
    return 'N';
  }
  if (direction == 'N') {
    return 'S';
  }
  if (direction == 'W') {
    return 'E';
  }
  if (direction == 'E') {
    return 'W';
  }
}
bool checkCenter(int x, int y) {
  if (x == MAZE_SIZE_X / 2 && y == MAZE_SIZE_Y / 2) return false;
  if (x == MAZE_SIZE_X / 2 && y == MAZE_SIZE_Y / 2 - 1) return false;
  if (x == MAZE_SIZE_X / 2 - 1 && y == MAZE_SIZE_Y / 2) return false;
  if (x == MAZE_SIZE_X / 2 - 1 && y == MAZE_SIZE_Y / 2 - 1) return false;
  return x > -1 && x < MAZE_SIZE_X && y > -1 && y < MAZE_SIZE_Y;
}

bool checkValid(int x, int y) {
  return x > -1 && x < MAZE_SIZE_X && y > -1 && y < MAZE_SIZE_Y;
}

void floodFill() {
  int x = 0, y = 0, direction = 'N', turn = 'F', prev = 100, centers = 0;
  int back = 0;
  while (true) {
    if(!path.empty() && path.top() == make_pair(x,y)){
      path.pop();
      API::clearColor(x, y);
    }
    else {
      path.push(make_pair(x,y));
      API::setColor(x, y, 'G');
    }
    if (arr[x][y] == 0)
      break;
      int mmin = prev;
      if (!API::wallRight()) {
        mmin = min(mmin, arr[getXY(x, y, direction, 'R', 0)]
                            [getXY(x, y, direction, 'R', 1)]);
      } else {
        wall[x][y][m[getNewDirection(direction, 'R')]] = 1;
      }
      // left
      if (!API::wallLeft()) {
        mmin = min(mmin, arr[getXY(x, y, direction, 'L', 0)]
                            [getXY(x, y, direction, 'L', 1)]);
      } else {
        wall[x][y][m[getNewDirection(direction, 'L')]] = 1;
      }
      // forward
      if (!API::wallFront()) {
        mmin = min(mmin, arr[getXY(x, y, direction, 'F', 0)]
                            [getXY(x, y, direction, 'F', 1)]);
      } else {
        wall[x][y][m[getNewDirection(direction, 'F')]] = 1;
      }
      // backward
      /* if (!wallBackward()) {
         mmin = min(
             mmin,
             arr[getXY(x, y, direction, 'B', 0)][getXY(x, y, direction, 'B',
       1)]);
       }*/
      log(to_string(x) + " " + to_string(y) + " " + to_string(mmin));
      if (arr[x][y] != mmin + 1) {
        arr[x][y] = mmin + 1;
        API::setText(x, y, std::to_string(arr[x][y]));
        log(std::to_string(x) + " " + std::to_string(y) + " " +
            to_string(arr[x][y]) + "\n");
        stack<pair<int, int>> st;
        if (checkCenter(x + 1, y) && !wall[x][y][m['E']])
          st.push({x + 1, y});
        if (checkCenter(x - 1, y) && !wall[x][y][m['W']])
          st.push({x - 1, y});
        if (checkCenter(x, y + 1) && !wall[x][y][m['N']])
          st.push({x, y + 1});
        if (checkCenter(x, y - 1) && !wall[x][y][m['S']])
          st.push({x, y - 1});
        log("start!!\n");
        log(std::to_string(st.size()) + "\n");
        while (!st.empty()) {
          pair<int, int> front = st.top();
          int xx = front.first, yy = front.second;
          st.pop();
          int mmmin = 100;
          if (checkValid(xx + 1, yy) && !wall[xx][yy][m['E']])
            mmmin = min(mmmin, arr[xx + 1][yy]);
          if (checkValid(xx - 1, yy) && !wall[xx][yy][m['W']])
            mmmin = min(mmmin, arr[xx - 1][yy]);
          if (checkValid(xx, yy + 1) && !wall[xx][yy][m['N']])
            mmmin = min(mmmin, arr[xx][yy + 1]);
          if (checkValid(xx, yy - 1) && !wall[xx][yy][m['S']])
            mmmin = min(mmmin, arr[xx][yy - 1]);
          if (arr[xx][yy] != mmmin + 1) {
            arr[xx][yy] = mmmin + 1;
            API::setText(xx, yy, std::to_string(arr[xx][yy]));
            if (checkCenter(xx + 1, yy) && !wall[xx][yy][m['E']])
              st.push({xx + 1, yy});
            if (checkCenter(xx - 1, yy) && !wall[xx][yy][m['W']])
              st.push({xx - 1, yy});
            if (checkCenter(xx, yy + 1) && !wall[xx][yy][m['N']])
              st.push({xx, yy + 1});
            if (checkCenter(xx, yy - 1) && !wall[xx][yy][m['S']])
              st.push({xx, yy - 1});
          }
        }
        log("Done!!\n");
      }
      int fmin = 100;
      if (!API::wallRight()) {
        fmin = min(fmin, arr[getXY(x, y, direction, 'R', 0)]
                            [getXY(x, y, direction, 'R', 1)]);
      }
      // left
      if (!API::wallLeft()) {
        fmin = min(fmin, arr[getXY(x, y, direction, 'L', 0)]
                            [getXY(x, y, direction, 'L', 1)]);
      }
      // forward
      if (!API::wallFront()) {
        fmin = min(fmin, arr[getXY(x, y, direction, 'F', 0)]
                            [getXY(x, y, direction, 'F', 1)]);
      }
      if (!API::wallFront() && fmin == arr[getXY(x, y, direction, 'F', 0)]
                                          [getXY(x, y, direction, 'F', 1)]) {
        API::moveForward();
        prev = arr[x][y];
        x = getXY(x, y, direction, 'F', 0);
        y = getXY(x, y, direction, 'F', 1);
        direction = getNewDirection(direction, 'F');
        turn = 'F';
        continue;
      }
      if (!API::wallRight() && fmin == arr[getXY(x, y, direction, 'R', 0)]
                                          [getXY(x, y, direction, 'R', 1)]) {
        back = 0;
        API::turnRight();
        API::moveForward();
        prev = arr[x][y];
        x = getXY(x, y, direction, 'R', 0);
        y = getXY(x, y, direction, 'R', 1);
        direction = getNewDirection(direction, 'R');
        turn = 'R';
        continue;
      }
      if (!API::wallLeft() && fmin == arr[getXY(x, y, direction, 'L', 0)]
                                         [getXY(x, y, direction, 'L', 1)]) {
        back = 0;
        API::turnLeft();
        API::moveForward();
        prev = arr[x][y];
        x = getXY(x, y, direction, 'L', 0);
        y = getXY(x, y, direction, 'L', 1);
        direction = getNewDirection(direction, 'L');
        turn = 'L';
        continue;
      }
      API::turnLeft();
      API::turnLeft();
      API::clearColor(x,y);
      path.pop();
      API::moveForward();
      prev = arr[x][y];
      x = getXY(x, y, direction, 'B', 0);
      y = getXY(x, y, direction, 'B', 1);
      direction = getNewDirection(direction, 'B');
      turn = 'B';
      back = 1;
      continue;
    }
  }

int main(int argc, char* argv[]) {
  log("Running...");
  API::setColor(0, 0, 'G');
  API::setText(0, 0, "abc");

  for (int y = 0; y < MAZE_SIZE_Y / 2; y += 1) {
    int distance = MAZE_SIZE_X / 2 + MAZE_SIZE_Y / 2 - 2 - y;
    for (int x = 0; x < MAZE_SIZE_X / 2; x += 1) {
      arr[x][y] = distance;
      arr[x][MAZE_SIZE_Y - 1 - y] = distance;
      arr[MAZE_SIZE_X - 1 - x][MAZE_SIZE_Y - 1 - y] = distance;
      arr[MAZE_SIZE_X - 1 - x][y] = distance;
      distance -= 1;
    }
  }
  FILE *ptr = fopen("input.txt","r");
  for (int i = 0; i < MAZE_SIZE_X; i++) {
    for (int j = 0; j < MAZE_SIZE_Y; j++) {
      fscanf(ptr,"%d",&arr[i][j]);
    }
  }
  fclose(ptr);
  for (int i = 0; i < MAZE_SIZE_X; i++) {
    for (int j = 0; j < MAZE_SIZE_Y; j++) {
      API::setText(i, j, std::to_string(arr[i][j]));
    }
  }
  // API::setColor(1,0,'G');
  m['N'] = 0;
  m['E'] = 1;
  m['S'] = 2;
  m['W'] = 3;
  floodFill();
  ofstream MyFile("input.txt");
  for (int i = 0; i < MAZE_SIZE_X; i++) {
    for (int j = 0; j < MAZE_SIZE_Y; j++) {
      MyFile << arr[i][j];
      MyFile << endl;
    }
  }
  MyFile.close();
}
