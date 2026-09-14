"""Compare diagnostic radii across libm implementations without changing meshes."""
from copy import deepcopy
import json

import numpy as np


def assert_goal_profile_equivalent(actual_bytes, expected_bytes):
    actual = json.loads(actual_bytes)
    expected = json.loads(expected_bytes)
    assert actual.keys() == expected.keys()
    actual_sections = actual['inner_sections']
    expected_sections = expected['inner_sections']
    assert len(actual_sections) == len(expected_sections)
    for actual_section, expected_section in zip(actual_sections, expected_sections):
        assert actual_section.keys() == expected_section.keys()
        assert isinstance(actual_section['inner'], list) and isinstance(expected_section['inner'], list)
        assert all(type(value) is float for value in actual_section['inner'])
        assert all(type(value) is float for value in expected_section['inner'])
        actual_radii = np.asarray(actual_section['inner'], dtype=np.float64)
        expected_radii = np.asarray(expected_section['inner'], dtype=np.float64)
        assert actual_radii.shape == expected_radii.shape
        assert np.isfinite(actual_radii).all() and np.isfinite(expected_radii).all()
        # Windows/Linux libm differed by at most three ULPs in these derived
        # metadata radii (~1.4e-17 m). The actual 12-decimal OBJ and every binary
        # STL still match byte-for-byte in the caller's separate assertions.
        np.testing.assert_array_max_ulp(actual_radii, expected_radii, maxulp=4)
        actual_section['inner'] = deepcopy(expected_section['inner'])
    # Heights, section counts, hashes, source labels, sampling settings, outer
    # geometry and every other field must remain exactly equal.
    assert actual == expected
