#! /usr/bin/env python

import sys

def main():
    next(sys.stdin)
    for line in sys.stdin:
        line = line.strip()
        line_parts = [x.strip() for x in line.split('\t')]
        assert len(line_parts) >= 2, line
        equation = line_parts[0]
        label = line_parts[1]
        eq_parts = [x.strip() for x in equation.split('=')]
        assert len(eq_parts) == 2 or len(eq_parts) == 3, line
        if len(eq_parts) == 2:
            dotprod = eq_parts[0]
        else:
            dotprod = eq_parts[1]
        prod_parts = [x.strip() for x in dotprod.split('+')]
        assert len(prod_parts) == 6, line
        features = []
        for i, prod_part in enumerate(prod_parts):
            prod = prod_part.split('*')
            assert len(prod) == 2, line
            feature = prod[1]
            if i == 5: # Percent of trajectory completed
                float_feat = float(feature)
                if float_feat < 1:
                    feature = '0'
                else:
                    feature = '1'
            features.append(feature)
        print '\t'.join(features + [label])

if __name__ == '__main__':
    main()
