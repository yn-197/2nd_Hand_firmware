#ifndef MODE_SELECT_HPP
#define MODE_SELECT_HPP

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

class ModeSelector {
public:
    ModeSelector(int mode_num);

    void selectMode();
    int getSelectedMode() const;
    void executeSelectedMode(); // モード実行用

private:

    int mode_num_;
    int mode_;
    int selected_mode_;
    uint8_t sw1_prev_;
    uint8_t sw2_prev_;
};

#ifdef __cplusplus
}
#endif

#endif /* MODE_SELECT_HPP */
