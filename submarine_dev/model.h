#pragma once
#include <stdarg.h>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class RandomForest {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        uint8_t votes[2] = { 0 };
                        // tree #1
                        if (x[1] <= 998.5) {
                            if (x[3] <= 399.5) {
                                if (x[2] <= 29.5) {
                                    if (x[3] <= 249.0) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[2] <= 9.5) {
                                            votes[0] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #2
                        if (x[3] <= 399.5) {
                            if (x[3] <= 249.5) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[0] <= 647.5) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[2] <= 29.5) {
                                        if (x[0] <= 852.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #3
                        if (x[0] <= 649.5) {
                            votes[0] += 1;
                        }

                        else {
                            if (x[1] <= 997.5) {
                                if (x[3] <= 399.5) {
                                    if (x[3] <= 249.0) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[0] <= 854.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                if (x[1] <= 998.5) {
                                    if (x[0] <= 805.0) {
                                        votes[1] += 1;
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        // tree #4
                        if (x[1] <= 997.5) {
                            if (x[0] <= 646.0) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[2] <= 29.5) {
                                    if (x[3] <= 399.5) {
                                        if (x[0] <= 855.0) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        else {
                            if (x[1] <= 999.0) {
                                if (x[3] <= 431.5) {
                                    votes[1] += 1;
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        // tree #5
                        if (x[1] <= 996.5) {
                            if (x[2] <= 29.5) {
                                if (x[2] <= 9.5) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[1] <= 390.5) {
                                        if (x[3] <= 401.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        if (x[0] <= 850.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            if (x[1] <= 998.5) {
                                if (x[0] <= 805.5) {
                                    if (x[1] <= 997.5) {
                                        if (x[3] <= 580.0) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        votes[1] += 1;
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        // tree #6
                        if (x[3] <= 399.5) {
                            if (x[0] <= 649.5) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[0] <= 851.0) {
                                    if (x[3] <= 248.5) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[1] <= 1040.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #7
                        if (x[1] <= 998.5) {
                            if (x[0] <= 647.5) {
                                votes[0] += 1;
                            }

                            else {
                                if (x[3] <= 399.5) {
                                    if (x[3] <= 249.0) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[1] <= 295.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #8
                        if (x[1] <= 998.5) {
                            if (x[3] <= 399.5) {
                                if (x[0] <= 855.0) {
                                    if (x[2] <= 29.5) {
                                        if (x[0] <= 645.5) {
                                            votes[0] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #9
                        if (x[1] <= 998.5) {
                            if (x[2] <= 29.5) {
                                if (x[3] <= 399.5) {
                                    if (x[2] <= 9.5) {
                                        votes[0] += 1;
                                    }

                                    else {
                                        if (x[3] <= 248.0) {
                                            votes[0] += 1;
                                        }

                                        else {
                                            votes[1] += 1;
                                        }
                                    }
                                }

                                else {
                                    votes[0] += 1;
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // tree #10
                        if (x[1] <= 998.5) {
                            if (x[3] <= 399.5) {
                                if (x[0] <= 647.5) {
                                    votes[0] += 1;
                                }

                                else {
                                    if (x[0] <= 854.5) {
                                        if (x[2] <= 30.5) {
                                            votes[1] += 1;
                                        }

                                        else {
                                            votes[0] += 1;
                                        }
                                    }

                                    else {
                                        votes[0] += 1;
                                    }
                                }
                            }

                            else {
                                votes[0] += 1;
                            }
                        }

                        else {
                            votes[0] += 1;
                        }

                        // return argmax of votes
                        uint8_t classIdx = 0;
                        float maxVotes = votes[0];

                        for (uint8_t i = 1; i < 2; i++) {
                            if (votes[i] > maxVotes) {
                                classIdx = i;
                                maxVotes = votes[i];
                            }
                        }

                        return classIdx;
                    }

                protected:
                };
            }
        }
    }