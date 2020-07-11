/***********************************************************************/
/*                                                                     */
/*  FILE        :CTimer.cpp                                            */
/*  DATE        :Jun 28, 2020                                          */
/*  DESCRIPTION :タイマークラス                                        */
/*                                                                     */
/*  This file is generated by Tatsuya Miyazaki                         */
/*                                                                     */
/***********************************************************************/

#ifndef _CTIMER_CPP__
#define _CTIMER_CPP__

#include "CTimer.hpp"

/***********************************************************************/
/*                           タイマー関数                              */
/*        注：millis()を使用する                                       */
/***********************************************************************/
// タイマーをクリアする
void CTimer::startTimer(void) {
  tempTimer = millis();
}

// タイマー値[ms]を取得する
uint32_t CTimer::getTime(void) {
  return(millis() - tempTimer);
}

#endif  // _CTIMER_CPP__