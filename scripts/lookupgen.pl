#!/usr/bin/perl
# dumps lookup tables for arduino and FPGA

use warnings;
use strict;

my $odir = $ARGV[0] || './';
my $afh = outfile('lookuptables.h');

camerax();
height();

# perpWallDist -> height map
# takes 10 bits of perpWallDist (>> 3)
# the smallest perpwalldist numbers give crazy heights anyway.
sub height {
    # 32 / 1024 (32 == max distance)
    # convert distance to wall height.
    my $scale = 0.03125;
    my $tot = $scale;
    print $afh "const uint16_t pdist_to_height_data[] PROGMEM = {\n";
    for (my $x = 0; $x < 1024; $x++) {
        my $cur = int(240 / $tot);
        # height
        $cur = 512 if ($cur > 512);
        # also the Q8.8 inverse of distance, for the floordist calc
        my $icur = int((1 / $tot) * 2**8);
        # texture scale
        my $tscale = int((32 << 8) / $cur);
        for ($cur, $tscale, $icur) {
            my $bin = pack('n', $_);
            my $str = hexstr($bin);
            print $afh "0x$str, ";

        }
        $tot += $scale;
    }
    print $afh "\n};\n";
}

sub camerax {
    print $afh "const uint16_t camerax_data[] PROGMEM = {\n";
    for (my $x = 0; $x < 320; $x++) {
        my $cur = 2 * $x / 320 - 1;
        my $num = int($cur * 2**8);
        my $bin = pack('n', int($cur * 2**8));
        my $str = hexstr($bin);
        print $afh "0x$str, ";
    }
    print $afh "\n};\n";
}

sub hexstr {
    my $b = shift;
    my $str = join("", map { sprintf("%02x", ord($_)) } split(//, $b));
    return $str;
}

sub outfile {
    my $f = shift;
    open(my $fh, "> $odir/$f") or die "couldn't open: $odir/$f for writing: $!";
    return $fh;
}


