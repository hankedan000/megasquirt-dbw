#include <unity.h>

#include "FaultFilter.h"

static const uint8_t LT_FAULT_THRESH = 3;
static const uint8_t ST_FAULT_TIMEOUT_MAX = 2;
static const uint8_t LT_FAULT_TIMEOUT_MAX = 5;

static FaultFilter f;

// process one sample and check the resulting mode and transition flag
#define ASSERT_PROCESS(expectMode, expectTransition, faulted) \
    { \
        const ModeWithTransition mwt = f.process(faulted); \
        TEST_ASSERT_EQUAL_INT_MESSAGE((int)FaultMode::expectMode, (int)mwt.mode, "mode"); \
        TEST_ASSERT_EQUAL_MESSAGE(expectTransition, mwt.transition, "transition"); \
    }

void setUp() {
    f.init(LT_FAULT_THRESH, ST_FAULT_TIMEOUT_MAX, LT_FAULT_TIMEOUT_MAX);
}

void tearDown() {}

void testNominal() {
    // should stay in nominal as long as no fault occur
    TEST_ASSERT_EQUAL_INT((int)FaultMode::Nominal, (int)f.mode());
    for (unsigned int i=0; i<10000; i++) {
        ASSERT_PROCESS(Nominal, false, false);
    }
}

void testShortTerm() {
    // enter into short term fault and make sure we clear it within 2 cycles
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, false);// timeout = 1
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0

    // enter into short term fault and trigger another fault while in it.
    // make sure the timeout gets extended for 1 additional short term fault window.
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, false);// timeout = 1
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, false);// timeout = 1
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0
}

void testLongTerm() {
    // enter into long term fault and make sure we clear it within 5 cycles
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(LongTerm, true, true);   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 4, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 3, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 2, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 1, ltFaultCnt = 3
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0, ltFaultCnt = 0

    // enter into long term fault and trigger another fault while in it.
    // make sure the timeout gets extended for 1 additional long term fault window.
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(LongTerm, true, true);   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 4, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 3, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, true);  // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 4, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 3, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 2, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 1, ltFaultCnt = 3
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0, ltFaultCnt = 0

    // enter into long term fault and make sure we stay there if faults persist
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(LongTerm, true, true);   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    // hit filter with a bunch of persistent faults
    for (unsigned int i=0; i<10000; i++) {
        ASSERT_PROCESS(LongTerm, false, true);
    }
}

void testReset() {
    // enter into long term fault and make sure a reset puts us back to nominal
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(LongTerm, true, true);   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 4, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 3, ltFaultCnt = 3
    f.reset();
    TEST_ASSERT_EQUAL_INT((int)FaultMode::Nominal, (int)f.mode());

    // -----------------------------------------
    // make sure filter is well behaved after the reset

    // enter into short term fault and make sure we clear it within 2 cycles
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, false);// timeout = 1
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0

    // enter into long term fault and make sure we clear it within 5 cycles
    ASSERT_PROCESS(Nominal, false, false);
    ASSERT_PROCESS(ShortTerm, true, true);  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_PROCESS(ShortTerm, false, true); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_PROCESS(LongTerm, true, true);   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 4, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 3, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 2, ltFaultCnt = 3
    ASSERT_PROCESS(LongTerm, false, false); // timeout = 1, ltFaultCnt = 3
    ASSERT_PROCESS(Nominal, true, false);   // timeout = 0, ltFaultCnt = 0
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(testNominal);
    RUN_TEST(testShortTerm);
    RUN_TEST(testLongTerm);
    RUN_TEST(testReset);
    return UNITY_END();
}
