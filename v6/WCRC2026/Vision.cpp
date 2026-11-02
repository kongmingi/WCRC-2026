#include "Vision.h"
#include "Hardware.h"

static_assert(MISSION_COUNT >= 1 && MISSION_COUNT <= MAX_ORDER, "MISSION must have 1..12 rows");

static int16_t GripX(bool lower)
{
  return lower ? CAM_GRIP_X_LOWER_PX : CAM_GRIP_X_UPPER_PX;
}

static bool InLayer(int16_t y, uint8_t layer)
{
  if (layer == 0) return y >= SCAN_LOWER_Y_MIN_PX && y <= SCAN_LOWER_Y_MAX_PX;
  if (y < SCAN_UPPER_Y_MIN_PX || y > SCAN_UPPER_Y_MAX_PX) return false;
  return layer == 2 ? y < SCAN_UPPER_SPLIT_Y_PX : y >= SCAN_UPPER_SPLIT_Y_PX;
}

static void SortInt(int16_t* v, uint8_t n)
{
  for (uint8_t i = 1; i < n; i++)
    for (uint8_t j = i; j > 0 && v[j - 1] > v[j]; j--) {
      int16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
    }
}

uint8_t ReadOrderFrame(uint8_t* seq)
{
  struct Seen { uint8_t sig; int16_t x; int16_t w; } seen[MAX_ORDER];
  uint8_t n = 0;
  if (pixy.ccc.getBlocks(true, PIXY_SIGMAP) < 0) return 0;

  for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& b = pixy.ccc.blocks[i];
    if (b.m_signature < 1 || b.m_signature > 6) continue;
    int16_t bx = b.m_x, by = b.m_y, bw = b.m_width;
    if (bx < ORDER_X_MIN_PX || bx > ORDER_X_MAX_PX) continue;
    if (by < ORDER_Y_MIN_PX || by > ORDER_Y_MAX_PX) continue;
    bool dup = false;
    for (uint8_t k = 0; k < n; k++) {
      if (seen[k].sig == b.m_signature && abs(seen[k].x - bx) <= ORDER_DUP_X_PX) {
        if (bw > seen[k].w) seen[k].w = bw;
        dup = true;
        break;
      }
    }
    if (dup || n >= MAX_ORDER) continue;
    seen[n].sig = b.m_signature;
    seen[n].x = b.m_x;
    seen[n].w = b.m_width;
    n++;
  }

  for (uint8_t i = 1; i < n; i++)
    for (uint8_t j = i; j > 0; j--) {
      bool swap = ORDER_REVERSE ? seen[j - 1].x < seen[j].x : seen[j - 1].x > seen[j].x;
      if (!swap) break;
      Seen t = seen[j]; seen[j] = seen[j - 1]; seen[j - 1] = t;
    }

  uint8_t count = 0;
  for (uint8_t i = 0; i < n; i++) {
    int16_t copies = (seen[i].w + ORDER_BLOCK_WIDTH_PX / 2) / ORDER_BLOCK_WIDTH_PX;
    if (copies < 1) copies = 1;
    for (int16_t c = 0; c < copies && count < MAX_ORDER; c++) seq[count++] = seen[i].sig;
  }
  return count;
}

static bool CountsMatch(const uint8_t* seq, uint8_t n)
{
  if (n != MISSION_COUNT) return false;
  for (uint8_t color = 1; color <= 6; color++) {
    uint8_t a = 0, b = 0;
    for (uint8_t i = 0; i < n; i++) if (seq[i] == color) a++;
    for (uint8_t i = 0; i < MISSION_COUNT; i++) if (MISSION[i].color == color) b++;
    if (a != b) return false;
  }
  return true;
}

uint8_t ReadOrder(uint8_t* out)
{
  uint8_t frames[ORDER_FRAMES][MAX_ORDER];
  uint8_t valid = 0;
  for (uint8_t f = 0; f < ORDER_FRAMES; f++) {
    AbortCheck();
    uint8_t n = ReadOrderFrame(frames[valid]);
    if (CountsMatch(frames[valid], n)) valid++;
    delay(20);
  }

  int8_t best = -1;
  uint8_t bestVotes = 0;
  for (uint8_t i = 0; i < valid; i++) {
    uint8_t votes = 0;
    for (uint8_t j = 0; j < valid; j++)
      if (memcmp(frames[i], frames[j], MISSION_COUNT) == 0) votes++;
    if (votes > bestVotes) { bestVotes = votes; best = i; }
  }
  if (best < 0 || bestVotes < ORDER_MIN_AGREE) return 0;
  memcpy(out, frames[best], MISSION_COUNT);
  return MISSION_COUNT;
}

void ScanView(bool lower, uint8_t* column)
{
  uint8_t votes[3][7];
  memset(votes, 0, sizeof(votes));
  int16_t cx = GripX(lower);

  for (uint8_t f = 0; f < SCAN_FRAMES; f++) {
    AbortCheck();
    if (pixy.ccc.getBlocks(true, PIXY_SIGMAP) < 0) continue;
    bool seen[3][7];
    memset(seen, 0, sizeof(seen));
    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      Block& b = pixy.ccc.blocks[i];
      if (b.m_signature < 1 || b.m_signature > 6) continue;
      if (abs((int16_t)b.m_x - cx) > CAM_COLUMN_HALF_WIDTH_PX) continue;
      for (uint8_t layer = lower ? 0 : 1; layer <= (lower ? 0 : 2); layer++)
        if (InLayer(b.m_y, layer)) seen[layer][b.m_signature] = true;
    }
    for (uint8_t l = 0; l < 3; l++)
      for (uint8_t s = 1; s <= 6; s++)
        if (seen[l][s]) votes[l][s]++;
  }

  for (uint8_t layer = lower ? 0 : 1; layer <= (lower ? 0 : 2); layer++) {
    uint8_t bestSig = INV_EMPTY, bestVotes = 0;
    for (uint8_t s = 1; s <= 6; s++)
      if (votes[layer][s] > bestVotes) { bestVotes = votes[layer][s]; bestSig = s; }
    column[layer] = bestVotes >= SCAN_MIN_HITS ? bestSig : INV_EMPTY;
  }
}

bool FindBlockX(uint8_t sig, uint8_t layer, int16_t& x)
{
  int16_t xs[SCAN_FRAMES];
  uint8_t n = 0;
  int16_t cx = GripX(layer == 0);
  for (uint8_t f = 0; f < SCAN_FRAMES; f++) {
    AbortCheck();
    if (pixy.ccc.getBlocks(true, 1 << (sig - 1)) < 0) continue;
    int16_t best = -1, bestErr = 9999;
    for (uint8_t i = 0; i < pixy.ccc.numBlocks; i++) {
      Block& b = pixy.ccc.blocks[i];
      if (b.m_signature != sig || !InLayer(b.m_y, layer)) continue;
      int16_t e = abs((int16_t)b.m_x - cx);
      if (e <= CAM_COLUMN_HALF_WIDTH_PX && e < bestErr) { bestErr = e; best = b.m_x; }
    }
    if (best >= 0) xs[n++] = best;
  }
  if (n < SCAN_MIN_HITS) return false;
  SortInt(xs, n);
  x = xs[n / 2];
  return true;
}

bool CellHas(uint8_t sig, uint8_t layer)
{
  int16_t x;
  return FindBlockX(sig, layer, x);
}

bool CenterOnBlock(uint8_t sig, uint8_t layer, float& movedRightMm)
{
  movedRightMm = 0;
  int16_t cx = GripX(layer == 0);
  uint32_t start = millis();
  while (millis() - start < CENTER_TIMEOUT_MS) {
    int16_t x;
    if (!FindBlockX(sig, layer, x)) return false;
    int16_t err = x - cx;
    if (abs(err) <= CENTER_TOL_PX) return true;
    float mm = constrain(abs(err) * CENTER_MM_PER_PX * CENTER_GAIN, CENTER_MIN_STEP_MM, CENTER_MAX_STEP_MM);
    DriveMm(err > 0 ? RIGHT : LEFT, mm, SPEED_CENTER_MMPS);
    movedRightMm += err > 0 ? mm : -mm;
  }
  Serial.println(F("center timeout"));
  return false;
}
