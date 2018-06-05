#! /usr/bin/env python

import sys
from sklearn.linear_model import LogisticRegression


def main():
    logistic_regression = LogisticRegression(fit_intercept=False, class_weight='balanced')
    X = []
    y = []
    for line in sys.stdin:
        line_parts = [float(x.strip()) for x in line.split('\t')]
        assert len(line_parts) == 7, line
        X.append(line_parts[:-1])
        y.append(line_parts[-1])
    logistic_regression.fit(X, y)
    print logistic_regression.coef_, logistic_regression.intercept_
    print logistic_regression.score(X, y)


if __name__ == '__main__':
    main()
