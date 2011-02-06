#!/usr/bin/perl
use strict;
use warnings;
use Data::Dumper;

open IN, "<WORLDcodes.c" or die "Fail $!";
my @source = <IN>;
close IN;

my %nacode;
for my $line (@source) {
    if ($line =~ /\s+\&(code_na\d\d\dCode), \/\/ same as \&(code_eu\d\d\dCode)/) {
	$nacode{$1} = $2;
    }
}

print Dumper \%nacode;
