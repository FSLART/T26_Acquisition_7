/*
 * Firmware version: fw_info block and version string. See fw_version.h.
 *
 * The shell already has Zephyr's "app version" and "app build-version" (CONFIG_APP_VERSION_SHELL,
 * on with the shell once the VERSION file exists).
 */

#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <zephyr/app_version.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "fw_version.h"

#define BUILD_STR STRINGIFY(APP_BUILD_VERSION)
#define HASH_HEX_LEN (2 * FW_HASH_LEN)

BUILD_ASSERT(sizeof(struct fw_info) == 48, "fw_info layout changed: bump FW_INFO_VERSION");
BUILD_ASSERT(sizeof(BUILD_STR) <= sizeof(((struct fw_info *) 0)->build), "APP_BUILD_VERSION too long for fw_info");
BUILD_ASSERT(APP_VERSION_MAJOR <= UINT8_MAX && APP_VERSION_MINOR <= UINT8_MAX && APP_PATCHLEVEL <= UINT8_MAX,
             "VERSION fields must fit in a byte");

/* __used: nothing in the firmware reads it, only a bootloader or updater */
__used __aligned(4) Z_GENERIC_SECTION(.fw_info) const struct fw_info fw_info = {
    .magic = FW_INFO_MAGIC,
    .info_version = FW_INFO_VERSION,
    .major = APP_VERSION_MAJOR,
    .minor = APP_VERSION_MINOR,
    .patch = APP_PATCHLEVEL,
    .tweak = APP_TWEAK,
    .build = BUILD_STR,
};

const char *fw_version_string(void) {
    return APP_VERSION_STRING " (" BUILD_STR ")";
}

static bool is_hex(const char *s, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!isxdigit((unsigned char) s[i])) {
            return false;
        }
    }
    return true;
}

int fw_version_parse_hash(const char *describe, uint8_t out[FW_HASH_LEN]) {
    if (describe == NULL) {
        return -EINVAL;
    }

    /* "<tag>-<n>-g<hash>": the hash follows the last "-g". Otherwise the whole string is the hash. */
    const char *hash = describe;
    for (const char *p = strstr(describe, "-g"); p != NULL; p = strstr(p + 1, "-g")) {
        hash = p + 2;
    }

    size_t len = strlen(hash);
    if (len < HASH_HEX_LEN || !is_hex(hash, len)) {
        return -EINVAL;
    }

    return hex2bin(hash, HASH_HEX_LEN, out, FW_HASH_LEN) == FW_HASH_LEN ? 0 : -EINVAL;
}

int fw_version_get_hash(uint8_t out[FW_HASH_LEN]) {
    return fw_version_parse_hash(fw_info.build, out);
}
