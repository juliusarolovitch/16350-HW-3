#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>
#include <regex>
#include <queue>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <stack>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
           unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions()
    {
        return this->actions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_init_cond() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_cond() const
    {
        return this->goal_conditions;
    }

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction()
    {
    }
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

struct GCComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs.toString() < rhs.toString();
    }
};

struct Node
{
    int g;
    int h;
    int f;
    shared_ptr<Node> parent;
    GroundedAction parent_action;
    set<GroundedCondition, GCComparator> conditions;

    Node(const set<GroundedCondition, GCComparator> &c, const int h, const int g, const int f, shared_ptr<Node> ptr) : conditions(c), g(g), f(f), h(h), parent(ptr) {}
};

struct NodeHasher
{
    size_t operator()(const set<GroundedCondition, GCComparator> &gc_set) const
    {
        string h;
        for (auto cond : gc_set)
        {
            h = cond.toString() + h;
        }
        return hash<string>{}(h);
    }
};

struct PlannerComparator
{
    bool operator()(const set<GroundedCondition, GCComparator> &lhs, const set<GroundedCondition, GCComparator> &rhs) const
    {
        return lhs == rhs;
    }
};

static auto compare = [](const shared_ptr<Node> &n1, const shared_ptr<Node> &n2)
{
    return n1->f > n2->f;
};

struct ActionNode
{
    unordered_map<GroundedCondition, shared_ptr<ActionNode>, GroundedConditionHasher, GroundedConditionComparator> multimap;
    vector<pair<unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>, GroundedAction>> value;
    bool end;

    ActionNode() : end(false) {}
};

priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, decltype(compare)> open(compare);
unordered_map<set<GroundedCondition, GCComparator>, shared_ptr<Node>, NodeHasher, PlannerComparator> nodes;
unordered_set<set<GroundedCondition, GCComparator>, NodeHasher, PlannerComparator> closed;

ActionNode rm;

int heuristic(
    set<GroundedCondition, GCComparator> &goal_state,
    set<GroundedCondition, GCComparator> &curr_state, bool use_heuristic)
{
    int h = 0;
    if (!use_heuristic)
        return h;
    for (GroundedCondition cond : goal_state)
        if (curr_state.find(cond) == curr_state.end())
            h++;
    return h;
}

list<GroundedAction> backtrack(std::__1::shared_ptr<Node> s, set<GroundedCondition, GCComparator> &goal)
{
    list<GroundedAction> plan;
    while (s)
    {
        plan.push_back(s->parent_action);
        s = s->parent;
    }
    plan.pop_back();
    plan.reverse();
    return plan;
}

list<GroundedAction> planner(Env *env)
{
    bool use_heuristic = true;
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    set<GroundedCondition, GCComparator> init, goal;
    for (auto val : env->get_init_cond())
        init.insert(val);
    for (auto val : env->get_goal_cond())
        goal.insert(val);

    unordered_set<Action, ActionHasher, ActionComparator> all_actions = env->get_actions();
    unordered_set<string> all_symbols = env->get_symbols();
    vector<string> sym;
    for (string s : all_symbols)
        sym.push_back(s);

    sort(sym.begin(), sym.end());

    vector<string> curr;
    vector<vector<string>> combos;
    set<GroundedCondition, GCComparator> preconds;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects;
    unordered_map<string, string> sym_map;

    for (auto &action : all_actions)
    {
        combos.clear();
        int num_args = action.get_args().size();
        int n = sym.size();
        int k = num_args;
        if (!(k <= 0 || k > n))
        {
            vector<int> indices(k);
            for (int i = 0; i < k; ++i)
            {
                indices[i] = i;
            }

            while (1)
            {
                vector<string> curr;
                for (int index : indices)
                {
                    curr.push_back(sym[index]);
                }
                combos.push_back(curr);

                int i = k - 1;
                while (i >= 0 && indices[i] == i + n - k)
                {
                    i--;
                }

                if (i < 0)
                    break;

                indices[i]++;
                for (int j = i + 1; j < k; ++j)
                {
                    indices[j] = indices[j - 1] + 1;
                }
            }
        }

        for (vector<string> symbol_seq : combos)
        {
            do
            {
                sym_map.clear();
                int ind = 0;
                list<string> ga_syms;
                for (string action_sym : action.get_args())
                {
                    sym_map[action_sym] = symbol_seq[ind];
                    ga_syms.push_back(symbol_seq[ind]);
                    ind++;
                }

                preconds.clear();
                for (Condition pc : action.get_preconditions())
                {
                    list<string> actual_args;
                    for (string pc_arg : pc.get_args())
                    {
                        if (sym_map.find(pc_arg) != sym_map.end())
                            actual_args.push_back(sym_map[pc_arg]);
                        else
                            actual_args.push_back(pc_arg);
                    }
                    GroundedCondition gc_precond(pc.get_predicate(), actual_args, pc.get_truth());
                    preconds.insert(gc_precond);
                }

                effects.clear();
                for (Condition eff : action.get_effects())
                {
                    list<string> actual_args;
                    for (string eff_arg : eff.get_args())
                    {
                        if (sym_map.find(eff_arg) != sym_map.end())
                            actual_args.push_back(sym_map[eff_arg]);
                        else
                            actual_args.push_back(eff_arg);
                    }
                    GroundedCondition gc_eff(eff.get_predicate(), actual_args, eff.get_truth());
                    effects.insert(gc_eff);
                }

                GroundedAction ga(action.get_name(), ga_syms);
                ActionNode *roadmap = &rm;
                for (auto it = preconds.begin(); it != preconds.end(); ++it)
                {
                    if (roadmap->multimap.find(*it) == roadmap->multimap.end())
                        roadmap->multimap[*it] = make_shared<ActionNode>();

                    if (next(it) == preconds.end())
                    {
                        roadmap = roadmap->multimap[*it].get();
                        roadmap->end = true;
                        roadmap->value.push_back({effects, ga});
                        break;
                    }
                    else
                        roadmap = roadmap->multimap[*it].get();
                }

            } while (next_permutation(symbol_seq.begin(), symbol_seq.end()));
        }
    }

    shared_ptr<Node> start = make_shared<Node>(init, heuristic(goal, init, use_heuristic), INT_MAX, INT_MAX, nullptr);
    start->g = 0;
    start->f = start->h;
    nodes[init] = start;
    open.push(start);

    int cost = 1;
    while (!open.empty())
    {
        shared_ptr<Node> s = open.top();

        if (closed.find(s->conditions) != closed.end())
            continue;
        closed.insert(s->conditions);
        bool goal_flag = true;
        for (auto gc : goal)
            if (s->conditions.find(gc) == s->conditions.end())
                goal_flag = false;

        if (goal_flag)
        {
            list<GroundedAction> plan;
            while (s)
            {
                plan.push_front(s->parent_action);
                s = s->parent;
            }
            plan.pop_front();
            chrono::steady_clock::time_point end = chrono::steady_clock::now();

            cout << endl
                 << "States expanded: " << closed.size() << endl;
            cout << "Time: " << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << " ms" << endl;

            return plan;
        }

        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effect;
        GroundedAction grounded_action;

        vector<ActionNode *> output;
        stack<pair<ActionNode *, set<GroundedCondition, GCComparator>::const_iterator>> stack;
        stack.push({&rm, s->conditions.begin()});

        while (!stack.empty())
        {
            auto curr = stack.top();
            stack.pop();

            for (auto end = s->conditions.end(); curr.second != end; ++curr.second)
            {
                if (curr.first->multimap.find(*curr.second) != curr.first->multimap.end())
                {
                    ActionNode *next_ptr = curr.first->multimap[*curr.second].get();
                    stack.push({next_ptr, next(curr.second)});
                }
            }

            if (!curr.first->value.empty())
                output.push_back(curr.first);
        }

        for (ActionNode *solution_ptr : output)
        {
            for (auto effect_action_pair : solution_ptr->value)
            {
                tie(effect, grounded_action) = effect_action_pair;
                set<GroundedCondition, GCComparator> prev_conds = s->conditions;
                for (auto e : effect)
                {
                    if (!e.get_truth())
                        prev_conds.erase(e);
                    else
                        prev_conds.insert(e);
                }

                if (closed.find(prev_conds) == closed.end())
                {
                    if (nodes.find(prev_conds) == nodes.end())
                    {
                        int h = heuristic(goal, prev_conds, use_heuristic);
                        shared_ptr<Node> newNode = make_shared<Node>(prev_conds, h, INT_MAX, INT_MAX, nullptr);
                        nodes[prev_conds] = newNode;
                    }
                    if (nodes[prev_conds]->g > (s->g + cost))
                    {
                        nodes[prev_conds]->g = s->g + cost;
                        nodes[prev_conds]->f = nodes[prev_conds]->g + nodes[prev_conds]->h;
                        nodes[prev_conds]->parent = s;
                        nodes[prev_conds]->parent_action = grounded_action;
                        open.push(nodes[prev_conds]);
                    }
                }
            }
        }
        open.pop();
    }

    return list<GroundedAction>{};
}

int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char *filename = (char *)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl
         << endl;
    Env *env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}
