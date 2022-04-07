#include "utils.h"

void split(String s, char delim, std::vector<String>& elems) {
    int pos = -1;
    while ((pos = s.indexOf(delim)) != -1) {
        elems.push_back(s.substring(0, pos));
        s.remove(0, pos + 1);
    }
    elems.push_back(s);
}