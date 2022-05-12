#include "id_qtuser_functions_epuck.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions() {
  RegisterUserFunction<CIDQTUserFunctions, CEPuckEntity>(
      &CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CEPuckEntity& c_entity) {
  /* The position of the text is expressed wrt the reference point of the e-puck
   * For a e-puck, the reference point is the center of its base.
   * See also the description in
   * $ argos3 -q e-puck
   */
  DrawText(CVector3(0.0, 0.0, 0.15),   // position
           c_entity.GetId().c_str());  // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions,
                                 "id_qtuser_functions_epuck")
