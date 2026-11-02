#include "types/Board.h"

namespace types {

Board::Board(double side, double doorWidth) : side(side), doorWidth(doorWidth) {
    const double left = (side - doorWidth) / 2;
    const double right = (side + doorWidth) / 2;
    walls = {{
        {{0, 0}, {left, 0}},
        {{right, 0}, {side, 0}},
        {{0, 0}, {0, side}},
        {{side, 0}, {side, side}},
        {{0, side}, {side, side}}
    }};
}

}
