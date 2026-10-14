#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

"""Unit tests for the Python API input validators (no game required).

These cover the client-side validation that rejects invalid inputs (inf/nan,
out-of-range coordinates, negative scale, malformed class paths) before they
reach the engine. They run fast and don't need a running Unreal instance.
"""

import pytest

from spear.utils.func_utils import (
    MAX_COORDINATE,
    _validate_finite_number,
    validate_class_path,
    validate_location,
    validate_scale,
)


# --- _validate_finite_number ------------------------------------------------

def test_finite_number_accepts_finite():
    _validate_finite_number(1.0)
    _validate_finite_number(-3.5)
    _validate_finite_number(0.0)
    _validate_finite_number(10**6)  # int is fine


def test_finite_number_accepts_bool():
    # bool is a subclass of int; explicitly allowed (not treated as a number)
    _validate_finite_number(True)
    _validate_finite_number(False)


@pytest.mark.parametrize("val", [float("inf"), float("-inf"), float("nan")])
def test_finite_number_rejects_non_finite(val):
    with pytest.raises(ValueError, match="Non-finite"):
        _validate_finite_number(val)


# --- validate_location ------------------------------------------------------

def test_location_accepts_valid():
    loc = {"X": 100.0, "Y": -200.0, "Z": 50.0}
    assert validate_location(loc) == loc


def test_location_accepts_at_limit():
    # Exactly at the renderer precision limit is allowed (abs(value) > MAX is the check)
    loc = {"X": MAX_COORDINATE, "Y": 0, "Z": 0}
    assert validate_location(loc) == loc


@pytest.mark.parametrize("val", [float("inf"), float("-inf"), float("nan")])
def test_location_rejects_non_finite(val):
    with pytest.raises(ValueError):
        validate_location({"X": val, "Y": 0, "Z": 0})


@pytest.mark.parametrize("val", [MAX_COORDINATE + 1, -(MAX_COORDINATE + 1), 1e9, -1e9])
def test_location_rejects_out_of_range(val):
    with pytest.raises(ValueError, match="exceeds the renderer"):
        validate_location({"X": val, "Y": 0, "Z": 0})


def test_location_ignores_non_numeric():
    # Non-numeric components are validated elsewhere; should not raise here
    loc = {"X": "a string", "Y": None, "Z": [1, 2]}
    assert validate_location(loc) == loc


def test_location_non_dict_passthrough():
    # A non-dict location is returned as-is (validated elsewhere)
    assert validate_location(None) is None
    assert validate_location(5) == 5


# --- validate_scale ---------------------------------------------------------

def test_scale_accepts_valid():
    scale = {"X": 1.0, "Y": 2.0, "Z": 0.5}
    assert validate_scale(scale) == scale


def test_scale_accepts_zero():
    scale = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    assert validate_scale(scale) == scale


def test_scale_rejects_negative():
    with pytest.raises(ValueError, match="Negative scale"):
        validate_scale({"X": -1.0, "Y": 1.0, "Z": 1.0})


@pytest.mark.parametrize("val", [float("inf"), float("nan")])
def test_scale_rejects_non_finite(val):
    with pytest.raises(ValueError):
        validate_scale({"X": val, "Y": 1.0, "Z": 1.0})


# --- validate_class_path ----------------------------------------------------

def test_class_path_accepts_bare_name():
    assert validate_class_path("AActor") == "AActor"
    assert validate_class_path("APawn") == "APawn"


def test_class_path_accepts_absolute():
    p = "/Game/Blueprints/MyActor.MyActor_C"
    assert validate_class_path(p) == p


def test_class_path_accepts_root():
    assert validate_class_path("/") == "/"


def test_class_path_rejects_malformed():
    with pytest.raises(ValueError, match="Invalid class path"):
        validate_class_path("NoSlash/Path")
    with pytest.raises(ValueError, match="Invalid class path"):
        validate_class_path("Game/Blueprints/MyActor")


def test_class_path_non_string_passthrough():
    # Non-string class paths (e.g. an UnrealClass object) are validated elsewhere
    obj = object()
    assert validate_class_path(obj) is obj
