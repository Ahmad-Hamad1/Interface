#include <bits/stdc++.h>
using namespace std;
int main() {
  ofstream MyFile("out.txt");
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      cout << j << " " << i << endl;
      int n, s, e, w;
      cin >> n >> e >> s >> w;
      MyFile << j <<" " << i << " "; 
      MyFile << n <<" "<< e << " " << s << " " << w <<endl;
      MyFile << endl;
    }
  }
    MyFile.close();
  return 0;
}