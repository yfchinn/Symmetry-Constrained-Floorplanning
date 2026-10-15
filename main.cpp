#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// ==========================================
// 1. Data Structure Definitions
// ==========================================

enum NodeType { NORMAL, HIERARCHY, CONTOUR, REPRESENTATIVE, SYM_PARTNER };

struct ContourSegment {
        int x, w, y;
        ContourSegment(int x, int w, int y) : x(x), w(w), y(y) {}
};

struct ContourInfo {
        int dx, w, dy;
};

struct Node {
        int id;
        string name;
        int orig_w, orig_h;
        bool rotated;

        int w, h;
        int x, y;
        int left, right, parent;
        NodeType type;

        bool is_self_sym;
        int macro_id;
        int counterpart_id;

        int local_w, local_h;
        int local_x, local_y;
        int local_left, local_right, local_parent;
        int local_root;

        bool is_horizontal_sym;
        vector<int> rep_nodes;

        // HB-Tree contour information
        vector<ContourInfo> bottom_contour;
        vector<ContourInfo> top_contours;
        ContourInfo top_seg;
        int hierarchy_id;

        Node()
            : id(-1),
              orig_w(0),
              orig_h(0),
              rotated(false),
              w(0),
              h(0),
              x(0),
              y(0),
              left(-1),
              right(-1),
              parent(-1),
              type(NORMAL),
              is_self_sym(false),
              macro_id(-1),
              counterpart_id(-1),
              local_w(0),
              local_h(0),
              local_x(0),
              local_y(0),
              local_left(-1),
              local_right(-1),
              local_parent(-1),
              local_root(-1),
              is_horizontal_sym(false),
              hierarchy_id(-1) {}
};

// ==========================================
// 2. Physical Design Floorplanner Class
// ==========================================

class Floorplanner {
    private:
        vector<Node> tree;
        map<string, int> name2id;
        int root_idx;
        long long best_cost;
        vector<Node> best_tree;

        list<ContourSegment> contour_list;
        list<ContourSegment> local_contour_list;

        mt19937 rng;

        // ==========================================
        // 3. Contour Maintenance (O(n) Packing Logic)
        // ==========================================
        void updateContour(list<ContourSegment>& contour, int start_x,
                           int width, int height_y) {
            if (width <= 0) return;
            int end_x = start_x + width;
            auto it = contour.begin();

            while (it != contour.end()) {
                int seg_start = it->x;
                int seg_end = it->x + it->w;

                if (seg_end <= start_x) {
                    ++it;
                    continue;
                }
                if (seg_start >= end_x) break;

                if (seg_start >= start_x && seg_end <= end_x) {
                    it = contour.erase(it);
                    continue;
                }
                if (seg_start < start_x && seg_end <= end_x) {
                    it->w = start_x - seg_start;
                    ++it;
                    continue;
                }
                if (seg_start >= start_x && seg_end > end_x) {
                    int shrink = end_x - seg_start;
                    it->w -= shrink;
                    it->x = end_x;
                    ++it;
                    continue;
                }
                if (seg_start < start_x && seg_end > end_x) {
                    it->w = start_x - seg_start;
                    int right_w = seg_end - end_x;
                    auto next_it = std::next(it);
                    contour.insert(next_it,
                                   ContourSegment(end_x, right_w, it->y));
                    ++it;
                    continue;
                }
            }

            auto insert_pos = contour.begin();
            while (insert_pos != contour.end() && insert_pos->x < start_x)
                ++insert_pos;
            contour.insert(insert_pos,
                           ContourSegment(start_x, width, height_y));
        }

        int findMaxYInContour(const list<ContourSegment>& contour, int x,
                              int w) {
            int max_y = 0;
            int end_x = x + w;
            for (const auto& seg : contour) {
                if (seg.x < end_x && seg.x + seg.w > x)
                    max_y = max(max_y, seg.y);
                if (seg.x >= end_x) break;
            }
            return max_y;
        }

        // ==========================================
        // 4. B*-tree Packing & Symmetry Island Logic
        // ==========================================
        void packLocalNode(int curr_idx, int parent_idx) {
            if (curr_idx == -1) return;
            Node& curr = tree[curr_idx];
            if (parent_idx == -1)
                curr.local_x = 0;
            else {
                Node& p = tree[parent_idx];
                curr.local_x = (p.local_left == curr_idx)
                                   ? p.local_x + p.local_w
                                   : p.local_x;
            }

            curr.local_y = findMaxYInContour(local_contour_list, curr.local_x,
                                             curr.local_w);
            updateContour(local_contour_list, curr.local_x, curr.local_w,
                          curr.local_y + curr.local_h);

            packLocalNode(curr.local_left, curr_idx);
            packLocalNode(curr.local_right, curr_idx);
        }

        void extractContours(int macro_id) {
            Node& m = tree[macro_id];
            vector<int> nodes;
            for (int r : m.rep_nodes) {
                nodes.push_back(r);
                if (tree[r].counterpart_id != -1)
                    nodes.push_back(tree[r].counterpart_id);
            }

            if (nodes.empty()) {
                m.bottom_contour.clear();
                m.top_contours.clear();
                return;
            }

            vector<int> xs;
            for (int id : nodes) {
                int actual_w =
                    tree[id].rotated ? tree[id].orig_h : tree[id].orig_w;
                xs.push_back(tree[id].x);
                xs.push_back(tree[id].x + actual_w);
            }
            sort(xs.begin(), xs.end());
            xs.erase(unique(xs.begin(), xs.end()), xs.end());

            vector<int> top_y(xs.size() - 1, 0);
            vector<int> bot_y(xs.size() - 1, 1e9);

            for (int id : nodes) {
                int x1 = tree[id].x;
                int actual_w =
                    tree[id].rotated ? tree[id].orig_h : tree[id].orig_w;
                int actual_h =
                    tree[id].rotated ? tree[id].orig_w : tree[id].orig_h;
                int x2 = x1 + actual_w;
                int y1 = tree[id].y;
                int y2 = y1 + actual_h;

                for (size_t i = 0; i < xs.size() - 1; ++i) {
                    if (xs[i] >= x1 && xs[i + 1] <= x2) {
                        top_y[i] = max(top_y[i], y2);
                        bot_y[i] = min(bot_y[i], y1);
                    }
                }
            }

            m.bottom_contour.clear();
            m.top_contours.clear();
            int curr_y = bot_y[0], start_x = xs[0];
            for (size_t i = 1; i < bot_y.size(); ++i) {
                if (bot_y[i] != curr_y) {
                    m.bottom_contour.push_back(
                        {start_x, xs[i] - start_x, curr_y});
                    curr_y = bot_y[i];
                    start_x = xs[i];
                }
            }
            m.bottom_contour.push_back({start_x, xs.back() - start_x, curr_y});

            curr_y = top_y[0];
            start_x = xs[0];
            for (size_t i = 1; i < top_y.size(); ++i) {
                if (top_y[i] != curr_y) {
                    m.top_contours.push_back(
                        {start_x, xs[i] - start_x, curr_y});
                    curr_y = top_y[i];
                    start_x = xs[i];
                }
            }
            m.top_contours.push_back({start_x, xs.back() - start_x, curr_y});
        }

        void packSymmetryIsland(int macro_id) {
            Node& m = tree[macro_id];
            local_contour_list.clear();
            local_contour_list.push_back(ContourSegment(-1000000, 2000000, 0));

            packLocalNode(m.local_root, -1);

            int global_min_x = 0, global_max_x = 0;
            for (int rep_id : m.rep_nodes) {
                Node& rep = tree[rep_id];
                int actual_w = rep.rotated ? rep.orig_h : rep.orig_w;
                int actual_h = rep.rotated ? rep.orig_w : rep.orig_h;

                if (rep.counterpart_id != -1) {
                    Node& other = tree[rep.counterpart_id];
                    if (!m.is_horizontal_sym) {
                        other.x = -rep.local_x - rep.local_w;
                        other.y = rep.local_y;
                        rep.x = rep.local_x;
                        rep.y = rep.local_y;
                    } else {
                        other.x = rep.local_x;
                        other.y = -rep.local_y - actual_h;
                        rep.x = rep.local_x;
                        rep.y = rep.local_y;
                    }
                    global_min_x = min({global_min_x, other.x, rep.x});
                    global_max_x = max(
                        {global_max_x, other.x + actual_w, rep.x + actual_w});
                } else {
                    if (!m.is_horizontal_sym) {
                        rep.x = -(actual_w / 2);
                        rep.y = rep.local_y;
                    } else {
                        rep.x = rep.local_x;
                        rep.y = -(actual_h / 2);
                    }
                    global_min_x = min(global_min_x, rep.x);
                    global_max_x = max(global_max_x, rep.x + actual_w);
                }
            }
            m.w = m.rep_nodes.empty() ? 0 : global_max_x - global_min_x;
            m.local_x = m.rep_nodes.empty() ? 0 : global_min_x;
        }

        void updateMacroContourNodes(int macro_id) {
            packSymmetryIsland(macro_id);
            extractContours(macro_id);

            vector<int> old_cnodes;
            int curr = tree[macro_id].right;
            while (curr != -1 && tree[curr].type == CONTOUR) {
                old_cnodes.push_back(curr);
                curr = tree[curr].left;
            }

            vector<ContourInfo> new_tc = tree[macro_id].top_contours;
            int M = old_cnodes.size();
            int N = new_tc.size();

            for (int i = 0; i < min(M, N); ++i)
                tree[old_cnodes[i]].top_seg = new_tc[i];

            if (N > M) {
                int prev_c = (M > 0) ? old_cnodes.back() : macro_id;
                for (int i = M; i < N; ++i) {
                    Node c;
                    c.type = CONTOUR;
                    c.name = "CONTOUR_" + to_string(tree.size());
                    c.id = tree.size();
                    c.hierarchy_id = macro_id;
                    c.top_seg = new_tc[i];

                    if (i == 0) {
                        int old_r = tree[macro_id].right;
                        tree[macro_id].right = c.id;
                        c.parent = macro_id;
                        c.left = old_r;
                        if (old_r != -1) tree[old_r].parent = c.id;
                    } else {
                        int old_l = tree[prev_c].left;
                        tree[prev_c].left = c.id;
                        c.parent = prev_c;
                        c.left = old_l;
                        if (old_l != -1) tree[old_l].parent = c.id;
                    }
                    prev_c = c.id;
                    tree.push_back(c);
                }
            } else if (N < M) {
                ContourInfo dummy_seg;
                if (N > 0) {
                    dummy_seg = new_tc[N - 1];
                    dummy_seg.dx += dummy_seg.w;
                    dummy_seg.w = 0;
                } else {
                    dummy_seg = {0, 0, 0};
                }

                for (int i = N; i < M; ++i) {
                    tree[old_cnodes[i]].top_seg = dummy_seg;
                }
            }
        }

        void packGlobalNode(int idx, int& max_global_x, int& max_global_y) {
            if (idx == -1) return;
            int p_id = tree[idx].parent;

            if (p_id == -1)
                tree[idx].x = 0;
            else if (tree[idx].type != CONTOUR) {
                tree[idx].x = (tree[p_id].left == idx)
                                  ? tree[p_id].x + tree[p_id].w
                                  : tree[p_id].x;
            }

            if (tree[idx].type == HIERARCHY) {
                int max_shift_y = 0;
                for (const auto& bc : tree[idx].bottom_contour) {
                    int needed_y = findMaxYInContour(
                        contour_list, tree[idx].x + bc.dx - tree[idx].local_x,
                        bc.w);
                    max_shift_y = max(max_shift_y, needed_y - bc.dy);
                }
                tree[idx].y = max_shift_y;
                max_global_x = max(max_global_x, tree[idx].x + tree[idx].w);

                packGlobalNode(tree[idx].left, max_global_x, max_global_y);
                packGlobalNode(tree[idx].right, max_global_x, max_global_y);

            } else if (tree[idx].type == CONTOUR) {
                int m_id = tree[idx].hierarchy_id;
                tree[idx].x =
                    tree[m_id].x + tree[idx].top_seg.dx - tree[m_id].local_x;
                tree[idx].w = tree[idx].top_seg.w;
                tree[idx].y = tree[m_id].y + tree[idx].top_seg.dy;

                updateContour(contour_list, tree[idx].x, tree[idx].w,
                              tree[idx].y);

                max_global_x = max(max_global_x, tree[idx].x + tree[idx].w);
                max_global_y = max(max_global_y, tree[idx].y);

                packGlobalNode(tree[idx].left, max_global_x, max_global_y);
                packGlobalNode(tree[idx].right, max_global_x, max_global_y);

            } else if (tree[idx].type == NORMAL) {
                tree[idx].y =
                    findMaxYInContour(contour_list, tree[idx].x, tree[idx].w);
                updateContour(contour_list, tree[idx].x, tree[idx].w,
                              tree[idx].y + tree[idx].h);

                max_global_x = max(max_global_x, tree[idx].x + tree[idx].w);
                max_global_y = max(max_global_y, tree[idx].y + tree[idx].h);

                packGlobalNode(tree[idx].left, max_global_x, max_global_y);
                packGlobalNode(tree[idx].right, max_global_x, max_global_y);
            }
        }

        void swapGlobalNodeContents(int n1, int n2) {
            swap(tree[n1].name, tree[n2].name);
            swap(tree[n1].orig_w, tree[n2].orig_w);
            swap(tree[n1].orig_h, tree[n2].orig_h);
            swap(tree[n1].rotated, tree[n2].rotated);
            swap(tree[n1].w, tree[n2].w);
            swap(tree[n1].h, tree[n2].h);
            swap(tree[n1].type, tree[n2].type);
            swap(tree[n1].rep_nodes, tree[n2].rep_nodes);
            swap(tree[n1].local_root, tree[n2].local_root);
            swap(tree[n1].is_horizontal_sym, tree[n2].is_horizontal_sym);

            if (tree[n1].type == HIERARCHY)
                for (int rep : tree[n1].rep_nodes) tree[rep].macro_id = n1;
            if (tree[n2].type == HIERARCHY)
                for (int rep : tree[n2].rep_nodes) tree[rep].macro_id = n2;
        }

        void deleteGlobalNode(int u) {
            if (u == -1) return;
            int p = tree[u].parent, l = tree[u].left, r = tree[u].right,
                rep = -1;
            if (l == -1 && r == -1)
                rep = -1;
            else if (l == -1)
                rep = r;
            else if (r == -1)
                rep = l;
            else {
                rep = l;
                int rm = l;
                while (tree[rm].right != -1) rm = tree[rm].right;
                tree[rm].right = r;
                if (r != -1) tree[r].parent = rm;
            }
            if (p == -1) {
                root_idx = rep;
                if (rep != -1) tree[rep].parent = -1;
            } else {
                if (tree[p].left == u)
                    tree[p].left = rep;
                else
                    tree[p].right = rep;
                if (rep != -1) tree[rep].parent = p;
            }
            tree[u].parent = tree[u].left = tree[u].right = -1;
        }

        void insertGlobalNode(int u, int v, bool as_left) {
            if (as_left) {
                int old = tree[v].left;
                tree[v].left = u;
                tree[u].parent = v;
                tree[u].left = old;
                if (old != -1) tree[old].parent = u;
                tree[u].right = -1;
            } else {
                int old = tree[v].right;
                tree[v].right = u;
                tree[u].parent = v;
                tree[u].right = old;
                if (old != -1) tree[old].parent = u;
                tree[u].left = -1;
            }
        }

        // ==========================================
        // 5. ASF-B*-tree (Local Tree) Perturbation
        // ==========================================

        void swapLocalNodeContents(int n1, int n2) {
            swap(tree[n1].name, tree[n2].name);
            swap(tree[n1].orig_w, tree[n2].orig_w);
            swap(tree[n1].orig_h, tree[n2].orig_h);
            swap(tree[n1].rotated, tree[n2].rotated);
            swap(tree[n1].local_w, tree[n2].local_w);
            swap(tree[n1].local_h, tree[n2].local_h);
            swap(tree[n1].is_self_sym, tree[n2].is_self_sym);
            swap(tree[n1].counterpart_id, tree[n2].counterpart_id);
        }

        void deleteLocalNode(int u, int macro_id) {
            if (u == -1) return;
            int p = tree[u].local_parent;
            int l = tree[u].local_left;
            int r = tree[u].local_right;
            int rep = -1;

            if (l == -1 && r == -1)
                rep = -1;
            else if (l == -1)
                rep = r;
            else if (r == -1)
                rep = l;
            else {
                rep = l;
                int rm = l;
                while (tree[rm].local_right != -1) rm = tree[rm].local_right;
                tree[rm].local_right = r;
                if (r != -1) tree[r].local_parent = rm;
            }

            if (p == -1) {
                tree[macro_id].local_root = rep;
                if (rep != -1) tree[rep].local_parent = -1;
            } else {
                if (tree[p].local_left == u)
                    tree[p].local_left = rep;
                else
                    tree[p].local_right = rep;
                if (rep != -1) tree[rep].local_parent = p;
            }
            tree[u].local_parent = tree[u].local_left = tree[u].local_right =
                -1;
        }

        void insertLocalNode(int u, int v, bool as_left, int macro_id) {
            if (v == -1) {
                tree[macro_id].local_root = u;
                return;
            }
            if (as_left) {
                int old = tree[v].local_left;
                tree[v].local_left = u;
                tree[u].local_parent = v;
                tree[u].local_left = old;
                if (old != -1) tree[old].local_parent = u;
                tree[u].local_right = -1;
            } else {
                int old = tree[v].local_right;
                tree[v].local_right = u;
                tree[u].local_parent = v;
                tree[u].local_right = old;
                if (old != -1) tree[old].local_parent = u;
                tree[u].local_left = -1;
            }
        }

        bool isLocalTreeValid(int macro_id) {
            Node& m = tree[macro_id];
            for (int u : m.rep_nodes) {
                if (tree[u].is_self_sym) {
                    int curr = m.local_root;
                    bool on_branch = false;
                    while (curr != -1) {
                        if (curr == u) {
                            on_branch = true;
                            break;
                        }
                        if (m.is_horizontal_sym) {
                            curr = tree[curr].local_left;
                        } else {
                            curr = tree[curr].local_right;
                        }
                    }
                    if (!on_branch) return false;
                }
            }
            return true;
        }

    public:
        Floorplanner() : root_idx(-1), best_cost(4e18) { rng.seed(42); }

        void parseInput(const string& filename) {
            ifstream fin(filename);
            if (!fin) {
                cerr << "Error: Cannot open input file!" << endl;
                exit(1);
            }

            string line;
            bool in_symmetry_section = false;

            while (getline(fin, line)) {
                if (line.empty()) continue;
                stringstream ss(line);
                string t1;
                if (!(ss >> t1)) continue;

                if (t1 == "NumBlocks:" || t1 == "NumBlock:") continue;
                if (t1 == "Symmetry") {
                    in_symmetry_section = true;
                    break;
                }

                Node n;
                ss >> n.orig_w >> n.orig_h;
                n.orig_w *= 2;
                n.orig_h *= 2;

                n.name = t1;
                n.w = n.orig_w;
                n.h = n.orig_h;
                n.id = tree.size();
                name2id[n.name] = n.id;
                tree.push_back(n);
            }

            if (in_symmetry_section) {
                Node macro;
                macro.type = HIERARCHY;
                macro.name = "SYM_MACRO_" + to_string(tree.size());
                auto close_macro = [&]() {
                    if (macro.rep_nodes.empty()) return;
                    int prev = -1;
                    for (int rep_id : macro.rep_nodes) {
                        if (prev == -1)
                            macro.local_root = rep_id;
                        else {
                            tree[prev].local_right = rep_id;
                            tree[rep_id].local_parent = prev;
                        }
                        prev = rep_id;
                    }
                    macro.id = tree.size();
                    tree.push_back(macro);
                };

                while (getline(fin, line)) {
                    if (line.empty()) continue;
                    stringstream ss(line);
                    string t1;
                    if (!(ss >> t1)) continue;

                    if (t1 == "Symmetry") {
                        close_macro();
                        macro = Node();
                        macro.type = HIERARCHY;
                        macro.name = "SYM_MACRO_" + to_string(tree.size());
                    } else {
                        string t2;
                        if (ss >> t2 && t2 != "//") {
                            int id1 = name2id[t1], id2 = name2id[t2];
                            tree[id1].type = SYM_PARTNER;
                            tree[id2].type = REPRESENTATIVE;
                            tree[id1].macro_id = tree[id2].macro_id =
                                tree.size();
                            tree[id2].counterpart_id = id1;
                            tree[id2].local_w = tree[id2].orig_w;
                            tree[id2].local_h = tree[id2].orig_h;
                            macro.rep_nodes.push_back(id2);
                        } else {
                            int id1 = name2id[t1];
                            tree[id1].type = REPRESENTATIVE;
                            tree[id1].macro_id = tree.size();
                            tree[id1].is_self_sym = true;
                            tree[id1].local_w = tree[id1].orig_w / 2;
                            tree[id1].local_h = tree[id1].orig_h;
                            macro.rep_nodes.push_back(id1);
                        }
                    }
                }
                close_macro();
            }
            fin.close();

            vector<int> global_nodes;
            vector<int> h_ids;
            for (size_t i = 0; i < tree.size(); ++i) {
                if (tree[i].type == NORMAL || tree[i].type == HIERARCHY) {
                    global_nodes.push_back(i);
                }
                if (tree[i].type == HIERARCHY) {
                    h_ids.push_back(i);
                }
            }

            auto buildBalancedTree = [&](auto& self, int l, int r) -> int {
                if (l > r) return -1;
                int mid = l + (r - l) / 2;
                int root = global_nodes[mid];

                int left_child = self(self, l, mid - 1);
                int right_child = self(self, mid + 1, r);

                tree[root].left = left_child;
                if (left_child != -1) tree[left_child].parent = root;

                tree[root].right = right_child;
                if (right_child != -1) tree[right_child].parent = root;

                return root;
            };

            if (!global_nodes.empty()) {
                root_idx = buildBalancedTree(buildBalancedTree, 0,
                                             global_nodes.size() - 1);
                if (root_idx != -1) tree[root_idx].parent = -1;
            } else {
                root_idx = -1;
            }

            for (int h_id : h_ids) {
                updateMacroContourNodes(h_id);
            }
        }

        long long evaluateCost() {
            contour_list.clear();
            contour_list.push_back(ContourSegment(-1000000, 2000000, 0));
            int max_x = 0, max_y = 0;

            packGlobalNode(root_idx, max_x, max_y);
            if (max_x == 0) return 0;

            long long area = ((long long)max_x * max_y) / 4;

            double ratio = (double)max_x / (double)max_y;

            double penalty_weight = 0.2; // 0.1 ~ 0.5
            double ratio_penalty = 0.0;
            double upper_bound = 2.0;
            double lower_bound = 0.5;
            if (ratio > upper_bound) {
                ratio_penalty = ratio - upper_bound;
            } else if (ratio < lower_bound) {
                ratio_penalty = lower_bound - ratio;
            }
            area += (long long)(area * penalty_weight * ratio_penalty);

            return area;
        }

        void solve() {
            auto start_time = chrono::high_resolution_clock::now();
            const int TIME_LIMIT_SECONDS = 3300;

            vector<int> local_reps;
            vector<int> macro_nodes;
            for (size_t i = 0; i < tree.size(); ++i) {
                if (tree[i].type == REPRESENTATIVE) local_reps.push_back(i);
                if (tree[i].type == HIERARCHY) macro_nodes.push_back(i);
            }

            int warmup_iters = 100;
            double total_uphill_cost = 0.0;
            int uphill_count = 0;

            long long current_warmup_cost = evaluateCost();

            for (int i = 0; i < warmup_iters; ++i) {
                vector<Node> backup_tree = tree;
                int root_backup = root_idx;

                // --- Perturbation ---
                bool is_local = (!local_reps.empty() && rng() % 5 != 0);

                if (is_local) {
                    int local_op = rng() % 4; // 0: Op1, 1: Op5, 2: Op2, 3: Op3
                    int target_macro_id = -1;

                    if (macro_nodes.empty()) {
                        local_op = 0;
                    } else {
                        if (local_op == 1 || local_op == 2 || local_op == 3) {
                            target_macro_id =
                                macro_nodes[rng() % macro_nodes.size()];

                            if ((local_op == 2 || local_op == 3) &&
                                tree[target_macro_id].rep_nodes.size() < 2) {
                                local_op = 0;
                            }
                        }
                    }

                    if (local_op == 0) {
                        int n1 = local_reps[rng() % local_reps.size()];
                        tree[n1].rotated = !tree[n1].rotated;
                        int aw = tree[n1].rotated ? tree[n1].orig_h
                                                  : tree[n1].orig_w;
                        int ah = tree[n1].rotated ? tree[n1].orig_w
                                                  : tree[n1].orig_h;

                        if (tree[n1].counterpart_id != -1) {
                            tree[n1].local_w = aw;
                            tree[n1].local_h = ah;
                            tree[tree[n1].counterpart_id].rotated =
                                tree[n1].rotated;
                        } else {
                            Node& macro = tree[tree[n1].macro_id];
                            if (macro.is_horizontal_sym) {
                                tree[n1].local_w = aw;
                                tree[n1].local_h = ah / 2;
                            } else {
                                tree[n1].local_w = aw / 2;
                                tree[n1].local_h = ah;
                            }
                        }
                        updateMacroContourNodes(tree[n1].macro_id);

                    } else if (local_op == 1) {
                        int macro_id = target_macro_id;
                        Node& macro = tree[macro_id];

                        macro.is_horizontal_sym = !macro.is_horizontal_sym;

                        for (int rep_id : macro.rep_nodes) {
                            Node& rep = tree[rep_id];

                            rep.rotated = !rep.rotated;
                            int aw = rep.rotated ? rep.orig_h : rep.orig_w;
                            int ah = rep.rotated ? rep.orig_w : rep.orig_h;

                            if (rep.counterpart_id != -1) {
                                rep.local_w = aw;
                                rep.local_h = ah;
                                tree[rep.counterpart_id].rotated = rep.rotated;
                            } else {
                                if (macro.is_horizontal_sym) {
                                    rep.local_w = aw;
                                    rep.local_h = ah / 2;
                                } else {
                                    rep.local_w = aw / 2;
                                    rep.local_h = ah;
                                }
                            }

                            swap(rep.local_left, rep.local_right);
                        }
                        updateMacroContourNodes(macro_id);

                    } else if (local_op == 2) {
                        int macro_id = target_macro_id;
                        Node& macro = tree[macro_id];
                        int u = macro.rep_nodes[rng() % macro.rep_nodes.size()];
                        int v = macro.rep_nodes[rng() % macro.rep_nodes.size()];

                        bool is_descendant = false;
                        int curr = v;
                        while (curr != -1) {
                            if (curr == u) {
                                is_descendant = true;
                                break;
                            }
                            curr = tree[curr].local_parent;
                        }

                        if (!is_descendant && u != v) {
                            deleteLocalNode(u, macro_id);
                            bool as_left = (rng() % 2 == 0);
                            insertLocalNode(u, v, as_left, macro_id);

                            if (!isLocalTreeValid(macro_id)) {
                                tree = backup_tree;
                                continue;
                            }
                            updateMacroContourNodes(macro_id);
                        } else {
                            tree = backup_tree;
                            continue;
                        }

                    } else if (local_op == 3) {
                        int macro_id = target_macro_id;
                        Node& macro = tree[macro_id];
                        int u = macro.rep_nodes[rng() % macro.rep_nodes.size()];
                        int v = macro.rep_nodes[rng() % macro.rep_nodes.size()];

                        if (u != v) {
                            swapLocalNodeContents(u, v);

                            if (!isLocalTreeValid(macro_id)) {
                                tree = backup_tree;
                                continue;
                            }
                            updateMacroContourNodes(macro_id);
                        } else {
                            tree = backup_tree;
                            continue;
                        }
                    }

                } else {
                    vector<int> active_normals;
                    vector<int> active_targets;
                    vector<int> stack;

                    if (root_idx != -1) stack.push_back(root_idx);
                    while (!stack.empty()) {
                        int curr = stack.back();
                        stack.pop_back();

                        if (tree[curr].type == NORMAL)
                            active_normals.push_back(curr);
                        active_targets.push_back(curr);

                        if (tree[curr].right != -1)
                            stack.push_back(tree[curr].right);
                        if (tree[curr].left != -1)
                            stack.push_back(tree[curr].left);
                    }

                    int op = rng() % 3;
                    if (op == 0 && !active_normals.empty()) {
                        int n1 = active_normals[rng() % active_normals.size()];
                        swap(tree[n1].w, tree[n1].h);
                        tree[n1].rotated = !tree[n1].rotated;
                    } else if (op == 1 && active_normals.size() >= 2) {
                        int n1 = active_normals[rng() % active_normals.size()];
                        int n2 = active_normals[rng() % active_normals.size()];
                        if (n1 != n2) swapGlobalNodeContents(n1, n2);
                    } else if (op == 2 && !active_normals.empty() &&
                               !active_targets.empty()) {
                        int u = active_normals[rng() % active_normals.size()];
                        int v = active_targets[rng() % active_targets.size()];

                        bool is_descendant = false;
                        int curr = v;
                        while (curr != -1) {
                            if (curr == u) {
                                is_descendant = true;
                                break;
                            }
                            curr = tree[curr].parent;
                        }

                        if (!is_descendant && u != v) {
                            deleteGlobalNode(u);
                            bool as_left = (rng() % 2 == 0);

                            if (tree[v].type == HIERARCHY) {
                                as_left = true;
                            } else if (tree[v].type == CONTOUR) {
                                as_left = false;
                            }

                            insertGlobalNode(u, v, as_left);
                        }
                    }
                }

                long long new_cost = evaluateCost();
                long long delta = new_cost - current_warmup_cost;

                if (delta > 0) {
                    total_uphill_cost += delta;
                    uphill_count++;
                }

                tree = backup_tree;
                root_idx = root_backup;
            }

            double delta_avg = (uphill_count > 0)
                                   ? (total_uphill_cost / uphill_count)
                                   : 100000.0;

            double P = 0.95;
            double temp = -delta_avg / log(P);
            int iter_per_temp = 20000;
            double cooling_rate = 0.9;
            double freeze_temp = 0.1;

            long long curr_cost = evaluateCost();
            best_cost = curr_cost;
            best_tree = tree;

            while (temp > freeze_temp) {
                auto current_time = chrono::high_resolution_clock::now();
                int elapsed_seconds = chrono::duration_cast<chrono::seconds>(
                                          current_time - start_time)
                                          .count();
                if (elapsed_seconds > TIME_LIMIT_SECONDS) {
                    cout << "\n[Warning] Time limit approaching ("
                         << elapsed_seconds
                         << "s)! Terminating SA early to guarantee output."
                         << endl;
                    break;
                }

                // cout << "current temp: " << temp << endl;

                for (int i = 0; i < iter_per_temp; ++i) {
                    if (i % 1000 == 0) {
                        auto inner_time = chrono::high_resolution_clock::now();
                        if (chrono::duration_cast<chrono::seconds>(inner_time -
                                                                   start_time)
                                .count() > TIME_LIMIT_SECONDS) {
                            break;
                        }
                    }

                    vector<Node> backup_tree = tree;
                    int root_backup = root_idx;

                    bool is_local = (!local_reps.empty() && rng() % 5 != 0);

                    if (is_local) {
                        int local_op =
                            rng() % 4; // 0: Op1, 1: Op5, 2: Op2, 3: Op3
                        int target_macro_id = -1;

                        if (macro_nodes.empty()) {
                            local_op = 0;
                        } else {
                            if (local_op == 1 || local_op == 2 ||
                                local_op == 3) {
                                target_macro_id =
                                    macro_nodes[rng() % macro_nodes.size()];

                                if ((local_op == 2 || local_op == 3) &&
                                    tree[target_macro_id].rep_nodes.size() <
                                        2) {
                                    local_op = 0;
                                }
                            }
                        }

                        if (local_op == 0) {
                            // Op1
                            int n1 = local_reps[rng() % local_reps.size()];
                            tree[n1].rotated = !tree[n1].rotated;
                            int aw = tree[n1].rotated ? tree[n1].orig_h
                                                      : tree[n1].orig_w;
                            int ah = tree[n1].rotated ? tree[n1].orig_w
                                                      : tree[n1].orig_h;

                            if (tree[n1].counterpart_id != -1) {
                                tree[n1].local_w = aw;
                                tree[n1].local_h = ah;
                                tree[tree[n1].counterpart_id].rotated =
                                    tree[n1].rotated;
                            } else {
                                Node& macro = tree[tree[n1].macro_id];
                                if (macro.is_horizontal_sym) {
                                    tree[n1].local_w = aw;
                                    tree[n1].local_h = ah / 2;
                                } else {
                                    tree[n1].local_w = aw / 2;
                                    tree[n1].local_h = ah;
                                }
                            }
                            updateMacroContourNodes(tree[n1].macro_id);

                        } else if (local_op == 1) {
                            // Op5
                            int macro_id = target_macro_id;
                            Node& macro = tree[macro_id];

                            macro.is_horizontal_sym = !macro.is_horizontal_sym;

                            for (int rep_id : macro.rep_nodes) {
                                Node& rep = tree[rep_id];

                                rep.rotated = !rep.rotated;
                                int aw = rep.rotated ? rep.orig_h : rep.orig_w;
                                int ah = rep.rotated ? rep.orig_w : rep.orig_h;

                                if (rep.counterpart_id != -1) {
                                    rep.local_w = aw;
                                    rep.local_h = ah;
                                    tree[rep.counterpart_id].rotated =
                                        rep.rotated;
                                } else {
                                    if (macro.is_horizontal_sym) {
                                        rep.local_w = aw;
                                        rep.local_h = ah / 2;
                                    } else {
                                        rep.local_w = aw / 2;
                                        rep.local_h = ah;
                                    }
                                }

                                swap(rep.local_left, rep.local_right);
                            }
                            updateMacroContourNodes(macro_id);

                        } else if (local_op == 2) {
                            // Op2
                            int macro_id = target_macro_id;
                            Node& macro = tree[macro_id];
                            int u =
                                macro.rep_nodes[rng() % macro.rep_nodes.size()];
                            int v =
                                macro.rep_nodes[rng() % macro.rep_nodes.size()];

                            bool is_descendant = false;
                            int curr = v;
                            while (curr != -1) {
                                if (curr == u) {
                                    is_descendant = true;
                                    break;
                                }
                                curr = tree[curr].local_parent;
                            }

                            if (!is_descendant && u != v) {
                                deleteLocalNode(u, macro_id);
                                bool as_left = (rng() % 2 == 0);
                                insertLocalNode(u, v, as_left, macro_id);

                                if (!isLocalTreeValid(macro_id)) {
                                    tree = backup_tree;
                                    continue;
                                }
                                updateMacroContourNodes(macro_id);
                            } else {
                                tree = backup_tree;
                                continue;
                            }

                        } else if (local_op == 3) {
                            // Op3
                            int macro_id = target_macro_id;
                            Node& macro = tree[macro_id];
                            int u =
                                macro.rep_nodes[rng() % macro.rep_nodes.size()];
                            int v =
                                macro.rep_nodes[rng() % macro.rep_nodes.size()];

                            if (u != v) {
                                swapLocalNodeContents(u, v);

                                if (!isLocalTreeValid(macro_id)) {
                                    tree = backup_tree;
                                    continue;
                                }
                                updateMacroContourNodes(macro_id);
                            } else {
                                tree = backup_tree;
                                continue;
                            }
                        }

                    } else {
                        vector<int> active_normals;
                        vector<int> active_targets;
                        vector<int> stack;

                        if (root_idx != -1) stack.push_back(root_idx);
                        while (!stack.empty()) {
                            int curr = stack.back();
                            stack.pop_back();

                            if (tree[curr].type == NORMAL)
                                active_normals.push_back(curr);
                            active_targets.push_back(curr);

                            if (tree[curr].right != -1)
                                stack.push_back(tree[curr].right);
                            if (tree[curr].left != -1)
                                stack.push_back(tree[curr].left);
                        }

                        int op = rng() % 3;
                        if (op == 0 && !active_normals.empty()) {
                            int n1 =
                                active_normals[rng() % active_normals.size()];
                            swap(tree[n1].w, tree[n1].h);
                            tree[n1].rotated = !tree[n1].rotated;
                        } else if (op == 1 && active_normals.size() >= 2) {
                            int n1 =
                                active_normals[rng() % active_normals.size()];
                            int n2 =
                                active_normals[rng() % active_normals.size()];
                            if (n1 != n2) swapGlobalNodeContents(n1, n2);
                        } else if (op == 2 && !active_normals.empty() &&
                                   !active_targets.empty()) {
                            int u =
                                active_normals[rng() % active_normals.size()];
                            int v =
                                active_targets[rng() % active_targets.size()];

                            bool is_descendant = false;
                            int curr = v;
                            while (curr != -1) {
                                if (curr == u) {
                                    is_descendant = true;
                                    break;
                                }
                                curr = tree[curr].parent;
                            }

                            if (!is_descendant && u != v) {
                                deleteGlobalNode(u);
                                bool as_left = (rng() % 2 == 0);

                                if (tree[v].type == HIERARCHY) {
                                    as_left = true;
                                } else if (tree[v].type == CONTOUR) {
                                    as_left = false;
                                }

                                insertGlobalNode(u, v, as_left);
                            }
                        }
                    }

                    long long new_cost = evaluateCost();
                    if (new_cost - curr_cost < 0 ||
                        exp(-(new_cost - curr_cost) / temp) >
                            ((double)rng() / rng.max())) {
                        curr_cost = new_cost;
                        if (curr_cost < best_cost) {
                            best_cost = curr_cost;
                            best_tree = tree;
                        }
                    } else {
                        tree = backup_tree;
                        root_idx = root_backup;
                    }
                }

                auto loop_time = chrono::high_resolution_clock::now();
                if (chrono::duration_cast<chrono::seconds>(loop_time -
                                                           start_time)
                        .count() > TIME_LIMIT_SECONDS) {
                    break;
                }

                temp *= cooling_rate;
            }

            tree = best_tree;
            evaluateCost();

            for (auto& m : tree) {
                if (m.type == HIERARCHY) {
                    for (int rep_id : m.rep_nodes) {
                        Node& rep = tree[rep_id];
                        if (rep.counterpart_id != -1) {
                            Node& other = tree[rep.counterpart_id];
                            other.x = m.x + (other.x - m.local_x);
                            other.y += m.y;
                        }
                        rep.x = m.x + (rep.x - m.local_x);
                        rep.y += m.y;
                    }
                }
            }
        }

        void writeOutput(const string& filename) {
            ofstream fout(filename);
            if (!fout) return;
            for (const auto& n : tree) {
                if (n.type == NORMAL || n.type == REPRESENTATIVE ||
                    n.type == SYM_PARTNER) {
                    fout << n.name << " " << (n.x / 2.0) << " " << (n.y / 2.0)
                         << " " << (n.rotated ? 1 : 0) << "\n";
                }
            }
            fout.close();
        }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_file> <output_file>\n";
        return 1;
    }
    Floorplanner fp;
    fp.parseInput(argv[1]);
    fp.solve();
    fp.writeOutput(argv[2]);
    return 0;
}