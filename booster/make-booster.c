#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <endian.h>

#define BOOSTER_BIN "booster.bin"

#define XXH_NO_LONG_LONG
#define XXH_FORCE_ALIGN_CHECK 0
#define XXH_FORCE_NATIVE_FORMAT 0
#define XXH_PRIVATE_API
#include "xxhash.h"

int main(int argc, char **argv) {
    int i;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s [infile] [outfile]\n", argv[0]);
        return 1;
    }

    char *infile_name = argv[1];
    char *outfile_name = argv[2];

    int booster_fd = open(BOOSTER_BIN, O_RDONLY);
    if (booster_fd == -1) {
        perror("Unable to open " BOOSTER_BIN);
        return 8;
    }

    int infile_fd = open(infile_name, O_RDONLY);
    if (infile_fd == -1) {
        perror("Unable to open input file");
        return 2;
    }

    int outfile_fd = open(outfile_name, O_WRONLY | O_CREAT | O_TRUNC, 0777);
    if (outfile_fd == -1) {
        perror("Unable to open output file");
        return 3;
    }

    struct stat stat_buf;
    if (-1 == fstat(infile_fd, &stat_buf)) {
        perror("Unable to determine size of input file");
        return 4;
    }

    // Copy the contents of the booster program
    while (1) {
        uint8_t b;
        if (sizeof(b) != read(booster_fd, &b, sizeof(b))) {
            break;
        }

        if (sizeof(b) != write(outfile_fd, &b, sizeof(b))) {
            perror("Unable to write to output file");
            return 10;
        }
    }

    uint8_t toboot_buffer[stat_buf.st_size];
    if (read(infile_fd, toboot_buffer, sizeof(toboot_buffer)) != sizeof(toboot_buffer)) {
        perror("Unable to read input file into RAM");
        return 11;
    }

    uint32_t in_size = htole32(stat_buf.st_size);
    if (sizeof(in_size) != write(outfile_fd, &in_size, sizeof(in_size))) {
        perror("Unable to write size to output file");
        return 5;
    }

    uint32_t in_hash = htole32(XXH32(toboot_buffer, sizeof(toboot_buffer), 0x12343210));
    if (sizeof(in_hash) != write(outfile_fd, &in_hash, sizeof(in_hash))) {
        perror("Unable to write hash to output file");
        return 12;
    }

    if (sizeof(toboot_buffer) != write(outfile_fd, toboot_buffer, sizeof(toboot_buffer))) {
        perror("Unable to write new bootloader to output file");
        return 7;
    }

    close(infile_fd);
    close(outfile_fd);
    close(booster_fd);
}