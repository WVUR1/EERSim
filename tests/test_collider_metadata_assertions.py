"""The cross-platform allowance is confined to derived radii and a few ULPs."""
from copy import deepcopy
import json
import math

import pytest

from collider_metadata_assertions import assert_goal_profile_equivalent


def profile():
    return {'source_sha256':'preserve this exact provenance',
            'inner_sections':[{'z':.0765,'inner':[.03619837502712144,.04]}],
            'mesh':{'triangles':5312},'outer_sections':[{'z':0,'outer':[.05]}]}


def compare(actual, expected):
    assert_goal_profile_equivalent(json.dumps(actual).encode(),json.dumps(expected).encode())


def test_three_ulp_radius_roundoff_is_permitted_without_mutating_inputs():
    expected=profile();actual=deepcopy(expected)
    for _ in range(3):
        actual['inner_sections'][0]['inner'][0]=math.nextafter(actual['inner_sections'][0]['inner'][0],0.)
    before=deepcopy(actual)
    compare(actual,expected)
    assert actual==before


@pytest.mark.parametrize('change', ['radius','height','provenance','triangles','nonfinite','section_count','radius_type'])
def test_geometry_provenance_and_numerically_significant_changes_are_rejected(change):
    expected=profile();actual=deepcopy(expected)
    if change=='radius': actual['inner_sections'][0]['inner'][0]+=1e-12
    elif change=='height': actual['inner_sections'][0]['z']=math.nextafter(.0765,1.)
    elif change=='provenance': actual['source_sha256']='different source'
    elif change=='triangles': actual['mesh']['triangles']+=1
    elif change=='nonfinite': actual['inner_sections'][0]['inner'][0]=float('nan')
    elif change=='radius_type': actual['inner_sections'][0]['inner'][0]=str(actual['inner_sections'][0]['inner'][0])
    elif change=='section_count': actual['inner_sections'].append(deepcopy(actual['inner_sections'][0]))
    with pytest.raises(AssertionError):
        compare(actual,expected)
