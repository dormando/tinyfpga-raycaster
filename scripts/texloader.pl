#!/usr/bin/perl
# dumps textures and palettes into a C structure to be .h'ed into the arduino
# project.
# each byte is two 4bit pixels.

use warnings;
use strict;

my $offset = 0;
my $poffset = 0;

my @textures = ();
my @palettes = ();

for my $file (@ARGV) {
    load_tex($file, $offset);
    my $pal = $file;
    $pal =~ s/\.flat/.pal/;
    load_pal($pal, $poffset);
    $offset += 1024;
    $poffset += 15;
    select undef, undef, undef, 0.25;
}

# the texture structure
print "const uint8_t texture_data[] PROGMEM = {\n";
for (@textures) {
    print $_, ",";
}
print "\n};\n";

# the palette structure.
print "const uint16_t palette_data[] PROGMEM = {\n";
for (@palettes) {
    print $_, ",";
}
print "\n};\n";

sub load_pal {
    my $file = shift;
    my $offset = shift;
    print "$file...\n";
    open(my $fh, "<$file") or die "boop: $!";
    my $buf;
    my @pbuf = ();
    my $cnt = 0;
    while (read($fh, $buf, 2)) {
        my $num = unpack('S', $buf);
        push(@pbuf, sprintf('0x%04x', $num));
        $cnt++;
    }
    close($fh);
    push(@palettes, sprintf("0x%04x", $cnt));
    push(@palettes, @pbuf);
}

sub load_tex {
    my $file = shift;
    my $offset = shift;
    print "$file...\n";
    open(my $fh, "<$file") or die "boop: $!";
    my $buf;
    my $hold = 0;
    my $half = 0;
    while (read($fh, $buf, 1)) {
        my $num = unpack('C', $buf);
        if ($half) {
            # combine the nybbles.
            # first number is at the front, so shifting or masking works to
            # retrieve.
            $hold |= ($num << 4);
            push(@textures, sprintf("0x%02x", $hold));
            $half = 0;
        } else {
            $half = 1;
            $hold = $num;
        }
    }
    close($fh);
}
