/*
 * Unit tests for src/fw_version.c: commit hash parsing and the fw_info block.
 *
 * tests/version/VERSION is a fixed 1.2.3+4 (not the firmware's VERSION), so the fw_info fields
 * can be checked. APP_BUILD_VERSION is this repository's real commit, so the build needs git.
 *
 * Run (Linux or WSL, from the repository folder):
 *   west twister -T tests -p native_sim/native/64
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "fw_version.h"

ZTEST_SUITE(fw_version_parse, NULL, NULL, NULL, NULL, NULL);

/* Plain hash, as git describe prints it when there is no annotated tag */
ZTEST(fw_version_parse, test_plain_hash) {
    const uint8_t expected[FW_HASH_LEN] = {0xee, 0x08, 0x15, 0xa1, 0xb2, 0xc3};
    uint8_t out[FW_HASH_LEN];

    zassert_ok(fw_version_parse_hash("ee0815a1b2c3", out));
    zassert_mem_equal(out, expected, FW_HASH_LEN);
}

/* Upper case and a hash longer than 12 characters (git lengthens it to stay unique) */
ZTEST(fw_version_parse, test_long_upper_case_hash) {
    const uint8_t expected[FW_HASH_LEN] = {0xee, 0x08, 0x15, 0xa1, 0xb2, 0xc3};
    uint8_t out[FW_HASH_LEN];

    zassert_ok(fw_version_parse_hash("EE0815A1B2C3D4", out));
    zassert_mem_equal(out, expected, FW_HASH_LEN);
}

/* After an annotated tag: "<tag>-<commits since>-g<hash>" */
ZTEST(fw_version_parse, test_hash_after_tag) {
    const uint8_t expected[FW_HASH_LEN] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab};
    uint8_t out[FW_HASH_LEN];

    zassert_ok(fw_version_parse_hash("v0.1.0-3-g0123456789ab", out));
    zassert_mem_equal(out, expected, FW_HASH_LEN);

    /* A tag name that itself contains "-g" */
    zassert_ok(fw_version_parse_hash("v1-gamma-12-g0123456789ab", out));
    zassert_mem_equal(out, expected, FW_HASH_LEN);
}

/* No hash: built without git, only a tag name (exactly on an annotated tag), too short, not hex */
ZTEST(fw_version_parse, test_no_hash) {
    uint8_t out[FW_HASH_LEN];

    zassert_equal(fw_version_parse_hash(NULL, out), -EINVAL);
    zassert_equal(fw_version_parse_hash("", out), -EINVAL);
    zassert_equal(fw_version_parse_hash("v0.1.0", out), -EINVAL);
    zassert_equal(fw_version_parse_hash("ee0815a", out), -EINVAL);
    zassert_equal(fw_version_parse_hash("ee0815a1b2cz", out), -EINVAL);
    zassert_equal(fw_version_parse_hash("ee0815a1b2c3-dirty", out), -EINVAL);
}

ZTEST_SUITE(fw_info, NULL, NULL, NULL, NULL, NULL);

/* Fields from tests/version/VERSION (1.2.3, tweak 4) */
ZTEST(fw_info, test_header_and_semver) {
    zassert_equal(fw_info.magic, FW_INFO_MAGIC);
    zassert_mem_equal(&fw_info.magic, "AQ7I", 4, "magic bytes in memory");
    zassert_equal(fw_info.info_version, FW_INFO_VERSION);
    zassert_equal(fw_info.major, 1);
    zassert_equal(fw_info.minor, 2);
    zassert_equal(fw_info.patch, 3);
    zassert_equal(fw_info.tweak, 4);
    zassert_equal(sizeof(fw_info), 48);
}

/* The build string is this repository's commit, and fw_version_string() carries it */
ZTEST(fw_info, test_build_is_commit_hash) {
    uint8_t out[FW_HASH_LEN];

    zassert_ok(fw_version_get_hash(out), "APP_BUILD_VERSION '%s' is not a git hash", fw_info.build);
    zassert_not_null(strstr(fw_version_string(), fw_info.build));
    zassert_equal(strncmp(fw_version_string(), "1.2.3 (", 7), 0, "%s", fw_version_string());
}
