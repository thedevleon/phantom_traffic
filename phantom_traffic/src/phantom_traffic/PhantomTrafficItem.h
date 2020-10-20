#pragma once

namespace phantom_traffic {

struct PhantomTrafficItem {
  int id;
  veins::Coord cs; //Coordinates
  double ct; //Time
  int cl; //Lane

  PhantomTrafficItem(int s_id, veins::Coord s_cs, double s_ct, int s_cl): id(s_id), cs(s_cs), ct(s_ct), cl(s_cl){}
  PhantomTrafficItem(): id(-1), cs(veins::Coord(0,0,0)), ct(0), cl(0){}
};

}
