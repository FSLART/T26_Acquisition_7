/*
 * Firmware version: semver from the VERSION file (bumped by CI) and the git commit it was built from.
 *
 * Both come from Zephyr's zephyr/app_version.h: APP_VERSION_MAJOR/MINOR/PATCHLEVEL/TWEAK and
 * APP_BUILD_VERSION (git describe --abbrev=12 --always, e.g. ee0815a1b2c3). The same data is stored in
 * fw_info, a fixed-format block right after the vector table, for a bootloader or the CAN updater.
 */

#ifndef T26_AQT7_FW_VERSION_H_
#define T26_AQT7_FW_VERSION_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/toolchain.h>

/* Bytes "AQ7I" at the start of the block (little-endian) */
#define FW_INFO_MAGIC 0x49375141u

/* Layout version of struct fw_info: bump it whenever the layout changes */
#define FW_INFO_VERSION 1

/* Bytes of the commit hash reported by fw_version_get_hash(): the 12 hex characters of APP_BUILD_VERSION */
#define FW_HASH_LEN 6

/*
 * Fixed-format block in flash, 48 bytes, little-endian. An updater finds it by searching for
 * FW_INFO_MAGIC (4-byte aligned) in the first 1 KiB of the image.
 */
struct fw_info {
    uint32_t magic;         /* FW_INFO_MAGIC */
    uint8_t info_version;   /* FW_INFO_VERSION */
    uint8_t major;          /* VERSION_MAJOR */
    uint8_t minor;          /* VERSION_MINOR */
    uint8_t patch;          /* PATCHLEVEL */
    uint32_t tweak;         /* VERSION_TWEAK */
    char build[32];         /* APP_BUILD_VERSION as text (the commit hash), NUL-terminated */
    uint8_t reserved[4];    /* 0, room for the updater (e.g. image size or CRC) */
} __packed;

extern const struct fw_info fw_info;

/* "0.1.0 (ee0815a1b2c3)": semver and commit, for the log and the shell */
const char *fw_version_string(void);

/*
 * Commit hash of this build as FW_HASH_LEN bytes (e.g. for a CAN frame).
 * @return 0, or -EINVAL if the build has no git hash (built without git)
 */
int fw_version_get_hash(uint8_t out[FW_HASH_LEN]);

/*
 * Hash bytes from a git describe string: "ee0815a1b2c3" or "v0.1.0-3-gee0815a1b2c3".
 * Uses the first 12 hex characters of the hash (git may print more to stay unique).
 * @return 0, or -EINVAL if there is no hash (e.g. "", "unknown", or only a tag name)
 */
int fw_version_parse_hash(const char *describe, uint8_t out[FW_HASH_LEN]);

#endif /* T26_AQT7_FW_VERSION_H_ */
