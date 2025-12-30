#!/usr/bin/env python3
"""
Test script to verify the improved text extraction.
"""

import requests
from bs4 import BeautifulSoup
from src.text_processor.text_cleaner import TextCleaner

# Fetch the main page
url = "https://robotic-book-murex.vercel.app/"
response = requests.get(url)
html_content = response.text

print("Original HTML length:", len(html_content))
print("\n" + "="*50 + "\n")

# Test with the improved cleaner
cleaner = TextCleaner()
improved_text = cleaner.clean_html_content(html_content)

print("Improved cleaning result length:", len(improved_text))
print("\nFirst 1000 characters of improved cleaned text:")
print(improved_text[:1000])
print("\n" + "="*50 + "\n")

print("Last 500 characters of improved cleaned text:")
print(improved_text[-500:])

# Count lines to see if we're preserving structure
lines = improved_text.split('\n')
print(f"\nTotal lines in cleaned content: {len(lines)}")
print("Sample lines (showing first 10):")
for i, line in enumerate(lines[:10]):
    print(f"  {i+1}: {line[:100]}{'...' if len(line) > 100 else ''}")