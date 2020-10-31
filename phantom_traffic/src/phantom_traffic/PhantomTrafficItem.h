#pragma once

namespace phantom_traffic {

struct PhantomTrafficItem {
  int id;
  veins::LAddress::L2Type sender_id;
  veins::Coord cs; //Coordinates
  double ct; //Time
  int cl; //Lane

  PhantomTrafficItem(int s_id, veins::LAddress::L2Type send_id, veins::Coord s_cs, double s_ct, int s_cl): id(s_id), sender_id(send_id), cs(s_cs), ct(s_ct), cl(s_cl){}
  PhantomTrafficItem(): id(-1), sender_id(veins::LAddress::L2NULL()), cs(veins::Coord(0,0,0)), ct(0), cl(0){}
};

}
